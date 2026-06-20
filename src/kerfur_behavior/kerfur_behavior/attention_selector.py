"""Attention selector: turns the perception stream into a gaze target.

Subscribes /head/detection, decides where Kerfur looks, publishes /kerfur/gaze.

Temperament (first build - "eager curious"):
  TRACK    - a detection is present -> look at it. The detector already picked
             the single most-salient target, so "track the most interesting
             thing / switch to something more exciting" is handled upstream;
             here we just follow it.
  ANTICIPATE - the target just left (label "") -> hold the last-seen gaze for a
             beat, "expecting it to come back." If a detection reappears, snap
             back to TRACK. If the hold expires, fall to SEARCH.
  SEARCH   - nothing present, anticipation expired -> lazy ambient drift, gaze
             wandering slowly around a modest range as if looking for something
             to land on. Any detection drops us straight back to TRACK.

No boredom / disengage timer: she stays locked while a target exists. The only
things that move attention are a more-salient target (decided upstream) or the
target leaving.

Output is a raw gaze target; the browser already smooths pupil motion and adds
autonomic saccades on top, so we do NOT interpolate here - we publish the
destination and let the renderer make it alive.

Coordinate note: image x and "eyes appear to follow you" x run OPPOSITE, because
the camera faces out like the face does and a tracking gaze must mirror the raw
image. So gaze_x = -detection_x by default (mirror_x param). y is not flipped.

Emotion-gating hook: PAD is not consumed yet. When emotion_engine moves to the
head, wander/track behavior can become mood-dependent (a sleepy Kerfur barely
engages, a playful one tracks eagerly). Left as a future tuning layer.
"""

import math
import random

import rclpy
from rclpy.node import Node

from kerfur_msgs.msg import Detection, GazeTarget


# Attention states
STATE_TRACK = "track"
STATE_ANTICIPATE = "anticipate"
STATE_SEARCH = "search"


class AttentionSelector(Node):
    def __init__(self):
        super().__init__("attention_selector")

        # --- Parameters (declared with defaults; set in kerfur_params.yaml) ---
        self.declare_parameter("publish_rate_hz", 20.0)
        # Mirror image-x into gaze-x. True for a forward-facing camera so the
        # eyes appear to follow the person rather than look away from them.
        self.declare_parameter("mirror_x", True)
        # How long to hold the last-seen gaze after a target leaves, before
        # giving up and wandering. The "where'd it go?" anticipation beat.
        self.declare_parameter("anticipate_hold_sec", 1.5)
        # --- Search-mode wander feel (lazy ambient drift) ---
        # Max gaze excursion while wandering. Kept modest so eyes don't pinball
        # to the corners; ~0.5 reads as natural looking-around.
        self.declare_parameter("wander_range", 0.5)
        # Seconds to ease from one wander point to the next (bigger = lazier).
        self.declare_parameter("wander_move_sec", 2.5)
        # Seconds to dwell at a wander point before picking a new one.
        self.declare_parameter("wander_pause_sec", 1.5)

        self.publish_rate = self.get_parameter("publish_rate_hz").value
        self.mirror_x = self.get_parameter("mirror_x").value
        self.anticipate_hold = self.get_parameter("anticipate_hold_sec").value
        self.wander_range = self.get_parameter("wander_range").value
        self.wander_move = self.get_parameter("wander_move_sec").value
        self.wander_pause = self.get_parameter("wander_pause_sec").value

        # --- State ---
        self.state = STATE_SEARCH
        self.last_gaze = (0.0, 0.0)         # last emitted gaze, for holding/easing
        self.anticipate_until = None        # clock time when anticipation expires

        # Wander sub-state: hold a target point for a while, then jump to a new
        # one. The browser smooths the jump; we never interpolate here.
        self.wander_to = self._random_wander_point()
        self.wander_phase_start = self.get_clock().now()

        # --- I/O ---
        self.create_subscription(Detection, "/head/detection", self.on_detection, 10)
        self.gaze_pub = self.create_publisher(GazeTarget, "/kerfur/gaze", 10)

        # Latest detection snapshot (written by callback, read by timer)
        self.latest_label = ""
        self.latest_x = 0.0
        self.latest_y = 0.0

        self.timer = self.create_timer(1.0 / self.publish_rate, self.tick)

        self.get_logger().info(
            f"attention_selector up: publishing /kerfur/gaze at {self.publish_rate}Hz, "
            f"mirror_x={self.mirror_x}, anticipate={self.anticipate_hold}s"
        )

    # ----- perception input -------------------------------------------------
    def on_detection(self, msg: Detection):
        self.latest_label = msg.label
        self.latest_x = msg.x
        self.latest_y = msg.y

    # ----- helpers ----------------------------------------------------------
    def _random_wander_point(self):
        r = self.wander_range
        return (random.uniform(-r, r), random.uniform(-r, r))

    def _gaze_from_detection(self):
        """Map the current detection's image coords to a gaze target."""
        gx = -self.latest_x if self.mirror_x else self.latest_x
        gy = self.latest_y
        return (gx, gy)

    def _now(self):
        return self.get_clock().now()

    def _secs_since(self, t):
        return (self._now() - t).nanoseconds / 1e9

    # ----- main loop --------------------------------------------------------
    def tick(self):
        have_target = bool(self.latest_label)

        if have_target:
            # Any detection -> TRACK, regardless of prior state. This handles
            # both "stay tracking" and "snap back during anticipation" and
            # "drop out of search the moment someone appears."
            self.state = STATE_TRACK
            self.anticipate_until = None
            gaze = self._gaze_from_detection()

        elif self.state == STATE_TRACK:
            # Target just disappeared -> begin anticipating its return.
            self.state = STATE_ANTICIPATE
            self.anticipate_until = self._now()
            gaze = self.last_gaze  # hold where we were looking

        elif self.state == STATE_ANTICIPATE:
            # Hold last gaze until the anticipation window expires.
            if self._secs_since(self.anticipate_until) >= self.anticipate_hold:
                # Give up waiting -> start wandering. Pick a first point and hold.
                self.state = STATE_SEARCH
                self.wander_to = self._random_wander_point()
                self.wander_phase_start = self._now()
                gaze = self.wander_to
            else:
                gaze = self.last_gaze

        else:  # STATE_SEARCH
            gaze = self._wander_step()

        self.last_gaze = gaze
        self._publish(gaze)

    def _wander_step(self):
        """Lazy ambient drift via DISCRETE targets: pick a point, hold it, pick
        a new one. We do NOT interpolate here - we emit a steady destination and
        let the browser's pupil smoothing animate the motion between points.
        (Interpolating here too made the selector and browser fight.)"""
        elapsed = self._secs_since(self.wander_phase_start)
        # Total time to spend on each point = travel allowance + dwell. We don't
        # actually move during travel; the browser does. We just hold the target
        # long enough for the browser to ease there and rest before the next hop.
        hold = self.wander_move + self.wander_pause
        if elapsed >= hold:
            self.wander_to = self._random_wander_point()
            self.wander_phase_start = self._now()
        return self.wander_to

    def _publish(self, gaze):
        msg = GazeTarget()
        msg.x = float(max(-1.0, min(1.0, gaze[0])))
        msg.y = float(max(-1.0, min(1.0, gaze[1])))
        msg.stamp = self._now().to_msg()
        self.gaze_pub.publish(msg)


def main():
    rclpy.init()
    node = AttentionSelector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
