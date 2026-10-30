"""Touch-bridge: reflex layer for capacitive touch.

Subscribes to the head MCU's zone state (sensor_msgs/Joy on /kerfur/touch_state,
published by the micro-ROS node kerfur_touch) and turns it into:

  * PADNudge on /kerfur/pad_nudge   - per-zone mood nudges, on touch-down and
                                      repeated while held
  * Expression on /kerfur/expression - a short ">w<" spike when the ears get
                                      worked repeatedly
  * optional mode_change            - sustained face contact -> "bonding",
                                      sustained ear contact  -> "playful"

Fast, no LLM. Zone order is fixed by the firmware:
  0 center, 1 cheek_l, 2 cheek_r, 3 ear_l, 4 ear_r
buttons[i] is the asserted state after the MCU's hysteresis and off-delay;
axes[i] is delta % over baseline (diagnostic only here).
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from rclpy.qos import qos_profile_sensor_data

from kerfur_msgs.msg import PADNudge, Expression

ZONES = ["center", "cheek_l", "cheek_r", "ear_l", "ear_r"]
FACE  = {0, 1, 2}
EARS  = {3, 4}


class TouchBridge(Node):
    def __init__(self):
        super().__init__("touch_bridge")

        # ---- per-zone nudge [dP, dA, dD] -------------------------------------
        defaults = {
            "center":  [0.20, -0.10, -0.05],   # head pat: calming, a little yielding
            "cheek_l": [0.25, -0.15,  0.00],   # cheek rub: most soothing contact
            "cheek_r": [0.25, -0.15,  0.00],
            "ear_l":   [0.20,  0.30,  0.05],   # ear scratch: pleasant AND activating
            "ear_r":   [0.20,  0.30,  0.05],
        }
        self.nudge = {}
        for z in ZONES:
            self.declare_parameter(f"nudge_{z}", defaults[z])
            self.nudge[z] = list(self.get_parameter(f"nudge_{z}").value)

        # ---- timing -----------------------------------------------------------
        self.declare_parameter("repeat_sec", 0.7)      # re-nudge interval while held
        self.declare_parameter("stale_sec", 1.0)       # no Joy for this long = all released
        self.repeat_sec = self.get_parameter("repeat_sec").value
        self.stale_sec  = self.get_parameter("stale_sec").value

        # ---- ear spike expression ---------------------------------------------
        self.declare_parameter("spike_expression", "happy")   # must match the face hub's name
        self.declare_parameter("spike_edges", 3)            # ear touch-downs ...
        self.declare_parameter("spike_window_sec", 2.5)     # ... within this window
        self.declare_parameter("spike_duration_sec", 1.5)
        self.declare_parameter("spike_refractory_sec", 4.0)
        self.spike_name   = self.get_parameter("spike_expression").value
        self.spike_edges  = self.get_parameter("spike_edges").value
        self.spike_window = self.get_parameter("spike_window_sec").value
        self.spike_dur    = self.get_parameter("spike_duration_sec").value
        self.spike_refr   = self.get_parameter("spike_refractory_sec").value

        # ---- mode changes (off until the engine has a way back out) -----------
        self.declare_parameter("enable_mode_change", False)
        self.declare_parameter("bonding_hold_sec", 8.0)     # cumulative face contact
        self.declare_parameter("playful_hold_sec", 6.0)     # cumulative ear contact
        self.declare_parameter("mode_cooldown_sec", 30.0)
        self.mode_enabled  = self.get_parameter("enable_mode_change").value
        self.bonding_hold  = self.get_parameter("bonding_hold_sec").value
        self.playful_hold  = self.get_parameter("playful_hold_sec").value
        self.mode_cooldown = self.get_parameter("mode_cooldown_sec").value

        # ---- state ------------------------------------------------------------
        self.held      = [False] * 5     # last reported state per zone
        self.last_fire = [0.0] * 5       # time of last nudge per zone
        self.last_joy  = None            # time of last Joy message
        self.ear_edges = []              # timestamps of recent ear touch-downs
        self.last_spike = -1e9
        self.face_acc  = 0.0             # cumulative contact seconds, decays when idle
        self.ear_acc   = 0.0
        self.last_mode = -1e9
        self.last_tick = None

        # ---- I/O --------------------------------------------------------------
        self.nudge_pub = self.create_publisher(PADNudge, "/kerfur/pad_nudge", 10)
        self.expr_pub  = self.create_publisher(Expression, "/kerfur/expression", 10)
        #self.create_subscription(Joy, "/kerfur/touch_state", self.on_joy, 10)
        self.create_subscription(Joy, "/kerfur/touch_state", self.on_joy, qos_profile_sensor_data)
        self.create_timer(0.1, self.tick)

        self.get_logger().info(
            "TouchBridge on /kerfur/touch_state; "
            + ", ".join(f"{z}={self.nudge[z]}" for z in ZONES)
        )

    # -------------------------------------------------------------------------
    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_joy(self, msg: Joy):
        t = self.now()
        self.last_joy = t
        if len(msg.buttons) < 5:
            return
        for i, z in enumerate(ZONES):
            on = bool(msg.buttons[i])
            if on and not self.held[i]:
                self._fire(i, z, t, edge=True)
                if i in EARS:
                    self.ear_edges.append(t)
            self.held[i] = on

    def tick(self):
        """Repeats while held, staleness, spike detection, mode accumulation."""
        t = self.now()
        dt = 0.0 if self.last_tick is None else t - self.last_tick
        self.last_tick = t

        # Link lost: treat every zone as released
        if self.last_joy is not None and t - self.last_joy > self.stale_sec:
            if any(self.held):
                self.get_logger().warn("touch_state stale - releasing all zones")
            self.held = [False] * 5

        # Repeat nudges while held
        for i, z in enumerate(ZONES):
            if self.held[i] and t - self.last_fire[i] >= self.repeat_sec:
                self._fire(i, z, t, edge=False)

        # Ear spike: N touch-downs inside the window, with refractory
        self.ear_edges = [e for e in self.ear_edges if t - e <= self.spike_window]
        if (len(self.ear_edges) >= self.spike_edges
                and t - self.last_spike >= self.spike_refr):
            self._spike(t)
            self.ear_edges.clear()

        # Contact accumulators: grow while held, drain at half rate when idle
        face_on = any(self.held[i] for i in FACE)
        ear_on  = any(self.held[i] for i in EARS)
        self.face_acc = min(self.face_acc + dt, self.bonding_hold) if face_on \
            else max(0.0, self.face_acc - 0.5 * dt)
        self.ear_acc  = min(self.ear_acc + dt, self.playful_hold) if ear_on \
            else max(0.0, self.ear_acc - 0.5 * dt)

        if self.mode_enabled and t - self.last_mode >= self.mode_cooldown:
            if self.ear_acc >= self.playful_hold:
                self._mode("playful", t); self.ear_acc = 0.0
            elif self.face_acc >= self.bonding_hold:
                self._mode("bonding", t); self.face_acc = 0.0

    # -------------------------------------------------------------------------
    def _fire(self, i, zone, t, edge):
        dp, da, dd = self.nudge[zone]
        n = PADNudge()
        n.d_pleasure  = float(dp)
        n.d_arousal   = float(da)
        n.d_dominance = float(dd)
        n.mode_change = ""
        n.reason = f"touch {zone} ({'down' if edge else 'held'})"
        self.nudge_pub.publish(n)
        self.last_fire[i] = t
        if edge:
            self.get_logger().info(f"{zone} down -> nudge [{dp:+.2f},{da:+.2f},{dd:+.2f}]")

    def _spike(self, t):
        e = Expression()
        e.name = self.spike_name
        e.intensity = 1.0
        e.duration_sec = float(self.spike_dur)
        self.expr_pub.publish(e)
        self.last_spike = t
        self.get_logger().info(f"ear spike -> expression '{self.spike_name}'")

    def _mode(self, mode, t):
        n = PADNudge()
        n.mode_change = mode
        n.reason = f"touch sustained -> {mode}"
        self.nudge_pub.publish(n)
        self.last_mode = t
        self.get_logger().info(f"mode_change -> {mode}")


def main():
    rclpy.init()
    node = TouchBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
