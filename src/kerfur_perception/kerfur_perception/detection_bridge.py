"""Detection bridge: reads the Hailo detector's socket and republishes the
most-salient detection as kerfur_msgs/Detection on /head/detection.

Companion to kerfur_detector.py (which runs in the Hailo venv, system Python).
That process does YOLO and writes newline-delimited JSON detections to a local
TCP socket; this node (in the ROS conda env) reads them and puts them on the bus.

Two-process split mirrors touch_bridge's native-handler pattern - the Hailo
binding can't live in the conda env, so the detector runs separately and feeds
us over a socket. Heavy pixels stay in the detector; only light conclusions
reach ROS.

Rate handling: the detector emits at camera rate (~30Hz). We do NOT publish that
fast - the behavior layer wants ~20Hz. So we stash the latest received detection
and publish on a ROS timer at publish_rate_hz, decoupled from the detector's
frame rate. Fresh-enough -> publish it; stale/absent -> publish label "" so the
attention selector idles rather than going stale-confident (CONVENTIONS §2).
"""

import json
import socket
import threading

import rclpy
from rclpy.node import Node

from kerfur_msgs.msg import Detection


class LatestFromSocket:
    """Lock-protected slot holding the most recent detection dict from the
    detector, plus a monotonic stamp for staleness aging."""

    def __init__(self):
        self._lock = threading.Lock()
        self._data = None
        self._mono_ns = 0

    def write(self, data, mono_ns):
        with self._lock:
            self._data = data
            self._mono_ns = mono_ns

    def read(self):
        with self._lock:
            return self._data, self._mono_ns


class DetectionBridge(Node):
    def __init__(self):
        super().__init__("detection_bridge")

        # --- Parameters (declared with defaults; set in kerfur_params.yaml) ---
        self.declare_parameter("detector_host", "127.0.0.1")
        self.declare_parameter("detector_port", 8766)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("freshness_timeout_sec", 0.4)

        self.host = self.get_parameter("detector_host").value
        self.port = self.get_parameter("detector_port").value
        self.publish_rate = self.get_parameter("publish_rate_hz").value
        self.freshness_timeout = self.get_parameter("freshness_timeout_sec").value

        self.slot = LatestFromSocket()
        self.det_pub = self.create_publisher(Detection, "/head/detection", 10)

        # Socket reader runs in a background thread so it never blocks ROS spin.
        self.reader_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.reader_thread.start()

        # ROS timer publishes the latest detection at our own (throttled) rate.
        self.timer = self.create_timer(1.0 / self.publish_rate, self.tick)

        self.get_logger().info(
            f"detection_bridge up: reading {self.host}:{self.port}, "
            f"publishing /head/detection at {self.publish_rate}Hz"
        )

    def _read_loop(self):
        """Connect to the detector and read newline-delimited JSON forever.
        Reconnects automatically if the detector isn't up yet or drops."""
        while rclpy.ok():
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.connect((self.host, self.port))
                self.get_logger().info("Connected to detector.")
                buf = ""
                while rclpy.ok():
                    data = sock.recv(4096)
                    if not data:
                        break  # detector closed
                    buf += data.decode("utf-8")
                    while "\n" in buf:
                        line, buf = buf.split("\n", 1)
                        line = line.strip()
                        if line:
                            self._on_line(line)
            except (OSError, ConnectionRefusedError) as e:
                self.get_logger().warn(
                    f"Detector not connected ({e}); retrying in 2s."
                )
            # back off before reconnecting
            self._sleep(2.0)

    def _sleep(self, sec):
        # Simple sleep that respects shutdown.
        import time
        t = 0.0
        while t < sec and rclpy.ok():
            time.sleep(0.1)
            t += 0.1

    def _on_line(self, line):
        try:
            data = json.loads(line)
        except json.JSONDecodeError:
            self.get_logger().warn(f"Bad JSON from detector: {line!r}")
            return
        now_ns = self.get_clock().now().nanoseconds
        self.slot.write(data, now_ns)

    def tick(self):
        """Publish the latest detection, or "" if stale/absent."""
        data, mono_ns = self.slot.read()

        fresh = False
        if data is not None and mono_ns > 0:
            age = (self.get_clock().now().nanoseconds - mono_ns) / 1e9
            fresh = age <= self.freshness_timeout

        msg = Detection()
        if fresh and data.get("label"):
            msg.label = str(data.get("label", ""))
            msg.x = float(data.get("x", 0.0))
            msg.y = float(data.get("y", 0.0))
            msg.size = float(data.get("size", 0.0))
            msg.confidence = float(data.get("confidence", 0.0))
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
    node = DetectionBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
