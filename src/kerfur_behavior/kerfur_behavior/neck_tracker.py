"""Neck tracker: turns the perception stream into a neck pose.

Subscribes /head/detection, publishes /kerfur/neck_pose (and the neck's
speed/accel multipliers on /kerfur/neck_dynamics) for the neck_controller
micro-ROS node on the ESP32. Replaces the old serial neck_bridge_poc: same
feel knobs, no serial port.

Why /head/detection and not /kerfur/gaze: the gaze stream is shaped for the
eyes. Its search-mode wander hops between discrete points and relies on the
browser to smooth them, which on the neck would be a head that pinballs.
The neck follows the detection and eases to neutral when it goes stale.

Pose message: geometry_msgs/Twist carrying degrees and millimetres relative to
the neck's rest pose (the firmware contract, NOT SI):
  angular.x = pitch deg, angular.y = roll deg, angular.z = yaw deg
  linear.x/y/z = lateral / fore-aft / height mm
Only yaw and pitch are driven here.

Two tracking modes (camera_moves_with_head):
  false - the camera is fixed to the body. Detection x/y is the person's
          bearing, so it maps straight to a yaw/pitch angle.
  true  - the camera rides on the head. Detection x/y is the remaining ERROR
          once the head has turned, so it is integrated: the head keeps
          turning until the person sits in the centre of the image.

Sign note: image x and head yaw run opposite for a forward-facing camera, so
yaw_sign defaults to -1. That flip lives here, in the consumer, never in the
firmware's geometric frame. If she turns away from you, flip the sign.

The firmware ignores poses while it is in maintenance and only moves once it
is homed and armed, so the last pose is re-sent once a second: the neck picks
it up as soon as it is armed, and the dynamics survive a firmware reboot.
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Vector3
from kerfur_msgs.msg import Detection


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class NeckTracker(Node):
    def __init__(self):
        super().__init__("neck_tracker")

        # --- Parameters (declared with defaults; set in kerfur_params.yaml) ---
        self.declare_parameter("publish_rate_hz", 25.0)
        self.declare_parameter("camera_moves_with_head", False)
        # Detection x/y = +/-1 maps to +/- this many degrees.
        self.declare_parameter("yaw_range_deg", 30.0)
        self.declare_parameter("pitch_range_deg", 12.0)   # kept tighter than yaw
        self.declare_parameter("yaw_sign", -1.0)
        self.declare_parameter("pitch_sign", -1.0)
        # Ignore detection jitter below this (detection units, 0..1).
        self.declare_parameter("dead_zone", 0.05)
        # Where in the image the detection centre should sit vertically
        # (detection units, + = below centre). 0 centres the box; a positive
        # value holds the box centre low in the frame, i.e. aims at the top of
        # a person box instead of its middle.
        self.declare_parameter("aim_y", 0.0)
        # Low-pass factor per tick; lower = lazier head.
        self.declare_parameter("smooth", 0.12)
        # camera_moves_with_head only: fraction of the remaining error closed
        # per second. Higher = snappier, too high = overshoot and hunting.
        self.declare_parameter("follow_gain", 1.5)
        # No detection for this long -> ease back to neutral.
        self.declare_parameter("stale_sec", 1.0)
        # Only publish when the pose moved by more than this.
        self.declare_parameter("min_delta_deg", 0.15)
        # Neck speed / accel multipliers; ambient tracking is unhurried.
        self.declare_parameter("speed_mult", 0.6)
        self.declare_parameter("accel_mult", 0.5)

        g = lambda n: self.get_parameter(n).value
        self.rate = g("publish_rate_hz")
        self.on_head = g("camera_moves_with_head")
        self.yaw_range = g("yaw_range_deg")
        self.pitch_range = g("pitch_range_deg")
        self.yaw_sign = g("yaw_sign")
        self.pitch_sign = g("pitch_sign")
        self.dead_zone = g("dead_zone")
        self.aim_y = g("aim_y")
        self.smooth = g("smooth")
        self.follow_gain = g("follow_gain")
        self.stale_sec = g("stale_sec")
        self.min_delta = g("min_delta_deg")
        self.speed_mult = g("speed_mult")
        self.accel_mult = g("accel_mult")

        # --- State (degrees, in detection sign convention until publish) ---
        self.tgt_yaw = self.tgt_pitch = 0.0
        self.cur_yaw = self.cur_pitch = 0.0
        self.sent_yaw = self.sent_pitch = None
        self.err_x = self.err_y = 0.0          # latest detection, head-camera mode
        self.last_det = None                   # clock time of last real detection

        # --- I/O ---
        self.create_subscription(Detection, "/head/detection", self.on_detection, 10)
        self.pose_pub = self.create_publisher(Twist, "/kerfur/neck_pose", 10)
        self.dyn_pub = self.create_publisher(Vector3, "/kerfur/neck_dynamics", 10)

        self.create_timer(1.0 / self.rate, self.tick)
        self.create_timer(1.0, self.keepalive)

        self.get_logger().info(
            f"neck_tracker up: /kerfur/neck_pose at {self.rate}Hz, "
            f"camera_moves_with_head={self.on_head}, "
            f"range +/-{self.yaw_range}/{self.pitch_range} deg"
        )

    # ----- perception input -------------------------------------------------
    def on_detection(self, msg: Detection):
        if msg.label == "":
            self.err_x = self.err_y = 0.0
            return                              # nothing salient: hold target
        x = clamp(msg.x, -1.0, 1.0)
        y = clamp(clamp(msg.y, -1.0, 1.0) - self.aim_y, -1.0, 1.0)
        if self.on_head:
            # Error signal; integrated in tick().
            self.err_x = x if abs(x) > self.dead_zone else 0.0
            self.err_y = y if abs(y) > self.dead_zone else 0.0
        else:
            # Bearing; only retarget when it moved out of the dead zone.
            if abs(x - self.tgt_yaw / self.yaw_range) > self.dead_zone:
                self.tgt_yaw = x * self.yaw_range
            if abs(y - self.tgt_pitch / self.pitch_range) > self.dead_zone:
                self.tgt_pitch = y * self.pitch_range
        self.last_det = self.get_clock().now()

    # ----- main loop --------------------------------------------------------
    def tick(self):
        stale = (self.last_det is None or
                 (self.get_clock().now() - self.last_det).nanoseconds / 1e9 > self.stale_sec)
        if stale:
            self.tgt_yaw = self.tgt_pitch = 0.0  # fail soft: ease to neutral
            self.err_x = self.err_y = 0.0
        elif self.on_head:
            k = self.follow_gain / self.rate
            self.tgt_yaw = clamp(self.tgt_yaw + self.err_x * self.yaw_range * k,
                                 -self.yaw_range, self.yaw_range)
            self.tgt_pitch = clamp(self.tgt_pitch + self.err_y * self.pitch_range * k,
                                   -self.pitch_range, self.pitch_range)

        self.cur_yaw += (self.tgt_yaw - self.cur_yaw) * self.smooth
        self.cur_pitch += (self.tgt_pitch - self.cur_pitch) * self.smooth

        if (self.sent_yaw is not None and
                abs(self.cur_yaw - self.sent_yaw) < self.min_delta and
                abs(self.cur_pitch - self.sent_pitch) < self.min_delta):
            return
        self._publish_pose()

    def keepalive(self):
        self.dyn_pub.publish(Vector3(x=float(self.speed_mult), y=float(self.accel_mult), z=0.0))
        self._publish_pose()

    def _publish_pose(self):
        self.sent_yaw, self.sent_pitch = self.cur_yaw, self.cur_pitch
        msg = Twist()
        msg.angular.z = float(self.yaw_sign * self.cur_yaw)
        msg.angular.x = float(self.pitch_sign * self.cur_pitch)
        self.pose_pub.publish(msg)


def main():
    rclpy.init()
    node = NeckTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
