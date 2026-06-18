"""Head perception node: runs the Hailo detector, publishes the single
most-salient semantic detection to /head/detection.

Architecture note (read before editing):
This node hosts TWO greedy loops in one process and keeps them in separate lanes.

  1. The Hailo GStreamer app owns ITS OWN loop. app.run() blocks forever and
     calls our app_callback on every frame, from ITS thread. The callback is
     blocking and must be FAST (CONVENTIONS allows no heavy work on a foreign
     callback thread) - so it only parses, picks the salient one, and stashes
     it under a lock. It never touches ROS.

  2. The ROS node runs normally (rclpy.spin on the main thread). A ROS timer at
     our own publish_rate reads the stashed detection and publishes Detection.
     It never touches GStreamer.

They share exactly one small piece of state (the 'latest' slot) through a lock.
This is the blackboard pattern applied at the thread level, and it gives us rate
decoupling for free: Hailo runs the callback at camera rate, we publish at
whatever rate the behavior layer wants.

Fail-soft (CONVENTIONS invariant #2): if no salient target is present, or the
last detection is older than freshness_timeout_sec, we publish label "" so the
attention selector idles/ages-out instead of going stale-confident. If the Hailo
stack is unavailable, the node logs and the ROS side still runs (publishing "").
"""

import threading

import rclpy
from rclpy.node import Node

from kerfur_msgs.msg import Detection


# --- Hailo imports -----------------------------------------------------------
# VERIFY THIS LINE against your installed package. The Hailo-10H master branch
# uses the current module layout below. Older Hailo-8L tutorials import from
# `hailo_apps_infra.detection_pipeline` / `hailo_rpi_common` - those are the OLD
# paths and will NOT match your 10H install. If import fails, run
#   python3 -c "import hailo_apps; print(hailo_apps.__file__)"
# and adjust the two import lines to match your tree.
HAILO_AVAILABLE = True
try:
    import hailo
    from hailo_apps.hailo_app_python.apps.detection.detection_pipeline import (
        GStreamerDetectionApp,
    )
    from hailo_apps.hailo_app_python.core.common.buffer_utils import (  # noqa: F401
        get_caps_from_pad,
    )
    from hailo_apps.hailo_app_python.core.gstreamer.gstreamer_app import (
        app_callback_class,
    )
except Exception:  # pragma: no cover - import shape varies by install
    HAILO_AVAILABLE = False

    class app_callback_class:  # minimal stand-in so the module still imports
        def __init__(self):
            self._count = 0

        def increment(self):
            self._count += 1

        def get_count(self):
            return self._count


# --- Shared state between the two loops --------------------------------------
class LatestDetection:
    """One lock-protected slot holding the most recent salient detection.

    The Hailo callback writes it; the ROS timer reads it. A monotonic stamp lets
    the ROS side decide staleness without trusting the callback's cadence.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._label = ""
        self._x = 0.0
        self._y = 0.0
        self._size = 0.0
        self._conf = 0.0
        self._mono_ns = 0  # when this was written, monotonic clock

    def write(self, label, x, y, size, conf, mono_ns):
        with self._lock:
            self._label = label
            self._x = x
            self._y = y
            self._size = size
            self._conf = conf
            self._mono_ns = mono_ns

    def read(self):
        with self._lock:
            return (self._label, self._x, self._y,
                    self._size, self._conf, self._mono_ns)


# --- The Hailo callback container --------------------------------------------
class HeadCallback(app_callback_class):
    """Holds tuning + the shared slot, and does the per-frame salience pick.

    Kept deliberately tiny: parse, choose, stash. No ROS, no logging in the hot
    path, nothing that could stall the GStreamer pipeline.
    """

    def __init__(self, slot, target_labels, conf_threshold, logger):
        super().__init__()
        self.slot = slot
        self.target_labels = target_labels   # set() of labels we care about
        self.conf_threshold = conf_threshold
        self._logger = logger
        self._clock = None  # set by the node so callback uses the same clock

    def set_clock(self, clock):
        self._clock = clock


def _bbox_center_and_size(det):
    """Convert a Hailo detection bbox (normalized 0..1) into:
       (cx_signed, cy_signed, area_fraction)
    where cx/cy are remapped to the project's [-1, +1] screen-relative range.
    """
    bbox = det.get_bbox()
    xmin = bbox.xmin()
    ymin = bbox.ymin()
    w = bbox.width()
    h = bbox.height()
    cx = xmin + w / 2.0          # 0..1
    cy = ymin + h / 2.0          # 0..1
    x_signed = 2.0 * cx - 1.0    # -1..+1, matches lookAt
    y_signed = 2.0 * cy - 1.0    # -1..+1
    area = max(0.0, min(1.0, w * h))  # fraction of frame, proximity proxy
    return x_signed, y_signed, area


def make_app_callback(cb: HeadCallback):
    """Build the frame callback closure Hailo will call on every buffer."""

    def app_callback(element, buffer, user_data):
        user_data.increment()
        if buffer is None:
            return

        roi = hailo.get_roi_from_buffer(buffer)
        detections = roi.get_objects_typed(hailo.HAILO_DETECTION)

        # Filter to the labels we care about and the confidence floor.
        best = None
        best_area = -1.0
        for det in detections:
            label = det.get_label()
            if cb.target_labels and label not in cb.target_labels:
                continue
            conf = det.get_confidence()
            if conf < cb.conf_threshold:
                continue
            x_signed, y_signed, area = _bbox_center_and_size(det)
            # Salience = largest box (closest / most prominent). Simple, sane
            # default; swap for confidence- or center-weighting later by tuning.
            if area > best_area:
                best_area = area
                best = (label, x_signed, y_signed, area, conf)

        now_ns = cb._clock.now().nanoseconds if cb._clock else 0
        if best is not None:
            label, x_signed, y_signed, area, conf = best
            cb.slot.write(label, x_signed, y_signed, area, conf, now_ns)
        # If nothing salient this frame, we do NOT overwrite the slot here -
        # the ROS timer ages it out via freshness_timeout_sec. This avoids
        # flicker from a single dropped frame.

    return app_callback


class PerceptionHead(Node):
    def __init__(self):
        super().__init__("perception_head")

        # --- Parameters (CONVENTIONS §6: declared with defaults here, set in
        #     kerfur_params.yaml under a matching 'perception_head' section) ---
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("freshness_timeout_sec", 0.4)
        self.declare_parameter("confidence_threshold", 0.5)
        # Empty list => accept ALL labels the model emits. Otherwise filter.
        self.declare_parameter("target_labels", ["person", "face"])
        # Hailo input source: "rpi" for CSI, or a /dev/videoN path for USB UVC.
        self.declare_parameter("hailo_input", "/dev/video0")
        # Optional explicit HEF path; "" => let Hailo pick by detected arch.
        self.declare_parameter("hef_path", "")

        self.publish_rate = self.get_parameter("publish_rate_hz").value
        self.freshness_timeout = self.get_parameter("freshness_timeout_sec").value
        conf_threshold = self.get_parameter("confidence_threshold").value
        target_labels = set(self.get_parameter("target_labels").value)
        self.hailo_input = self.get_parameter("hailo_input").value
        self.hef_path = self.get_parameter("hef_path").value

        # --- Shared slot + publisher ---
        self.slot = LatestDetection()
        self.det_pub = self.create_publisher(Detection, "/head/detection", 10)

        # --- The Hailo callback container ---
        self.cb = HeadCallback(
            self.slot, target_labels, conf_threshold, self.get_logger()
        )
        self.cb.set_clock(self.get_clock())

        # --- ROS timer: reads the slot, publishes Detection at our own rate ---
        self.timer = self.create_timer(1.0 / self.publish_rate, self.tick)

        # --- Start the Hailo app in its own thread (it owns a blocking loop) ---
        self.hailo_thread = None
        if HAILO_AVAILABLE:
            self.hailo_thread = threading.Thread(
                target=self._run_hailo, daemon=True
            )
            self.hailo_thread.start()
            self.get_logger().info(
                f"perception_head up: publishing /head/detection at "
                f"{self.publish_rate}Hz, targets={sorted(target_labels) or 'ALL'}"
            )
        else:
            self.get_logger().warn(
                "Hailo stack not importable - perception_head will publish "
                "empty detections (label \"\") and stay fail-soft. Verify the "
                "hailo imports at the top of perception_head.py against your "
                "installed package."
            )

    def _run_hailo(self):
        """Background thread: construct and run the blocking Hailo app."""
        try:
            app_callback = make_app_callback(self.cb)
            # GStreamerDetectionApp parses sys.argv-style options internally;
            # we pass input (and optional hef) through its expected mechanism.
            # NOTE: exact constructor args vary slightly by package version -
            # this is the second thing to verify on your install. The common
            # forms are GStreamerDetectionApp(app_callback, user_data) with
            # input/hef supplied via CLI/env, OR an options object. Adjust here.
            app = GStreamerDetectionApp(app_callback, self.cb)
            app.run()  # blocks until pipeline shutdown
        except Exception as e:  # fail soft - ROS side keeps publishing ""
            self.get_logger().error(
                f"Hailo app stopped ({e}). perception_head now publishing empty "
                f"detections only."
            )

    def tick(self):
        """Publish the latest salient detection, or "" if stale/absent."""
        label, x, y, size, conf, mono_ns = self.slot.read()

        # Staleness check: if the last write is older than the timeout (or there
        # was never one), publish the empty sentinel so the selector idles.
        fresh = False
        if mono_ns > 0:
            age_sec = (self.get_clock().now().nanoseconds - mono_ns) / 1e9
            fresh = age_sec <= self.freshness_timeout

        msg = Detection()
        if fresh:
            msg.label = label
            msg.x = float(x)
            msg.y = float(y)
            msg.size = float(size)
            msg.confidence = float(conf)
        else:
            msg.label = ""
            msg.x = 0.0
            msg.y = 0.0
            msg.size = 0.0
            msg.confidence = 0.0
        msg.stamp = self.get_clock().now().to_msg()
        self.det_pub.publish(msg)


def main():
    rclpy.init()
    node = PerceptionHead()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
