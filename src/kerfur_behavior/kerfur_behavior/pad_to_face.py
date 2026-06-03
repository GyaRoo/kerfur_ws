"""Behavior node: classifies PAD state into one of 8 octants + neutral, publishes Expression."""

import rclpy
from rclpy.node import Node

from kerfur_msgs.msg import PADState, Expression


# Octant labels indexed by binary (P, A, D) sign bits
# Index = 4*p_pos + 2*a_pos + d_pos
OCTANT_LABELS = [
    "bored",       # (-,-,-)
    "disdainful",  # (-,-,+)
    "anxious",     # (-,+,-)
    "angry",       # (-,+,+)
    "docile",      # (+,-,-)
    "relaxed",     # (+,-,+)
    "surprised",   # (+,+,-)
    "joyful",      # (+,+,+)
]


def classify_octant(p, a, d, hyst, prev_signs):
    """Classify PAD position into one of 8 octants, plus neutral.
    
    Returns (label, signs_tuple). signs_tuple is (p_sign, a_sign, d_sign)
    where each is 0 or 1, for use as the next call's prev_signs.
    
    Hysteresis: an axis only flips sign once it has crossed past zero
    by `hyst`. If currently positive (sign=1), must drop below -hyst to flip.
    """
    def axis_sign(value, prev_sign):
        if prev_sign == 1:
            # Currently positive, need to drop below -hyst to flip negative
            return 1 if value > -hyst else 0
        else:
            # Currently negative, need to rise above +hyst to flip positive
            return 1 if value > hyst else 0
    
    p_sign = axis_sign(p, prev_signs[0])
    a_sign = axis_sign(a, prev_signs[1])
    d_sign = axis_sign(d, prev_signs[2])
    
    # Check for neutral zone - all three axes near zero
    if abs(p) < hyst and abs(a) < hyst and abs(d) < hyst:
        return "neutral", (p_sign, a_sign, d_sign)
    
    idx = 4 * p_sign + 2 * a_sign + d_sign
    return OCTANT_LABELS[idx], (p_sign, a_sign, d_sign)


class PADToExpression(Node):
    def __init__(self):
        super().__init__("pad_to_expression")
        
        # Parameters
        self.declare_parameter("hysteresis", 0.1)
        self.declare_parameter("connection_timeout_sec", 2.0)
        self.hysteresis = self.get_parameter("hysteresis").value
        self.timeout = self.get_parameter("connection_timeout_sec").value
        
        # State
        self.current_pad = (0.0, 0.0, 0.0)
        self.current_mode = "default"
        self.prev_signs = (0, 0, 0)  # tracks hysteresis baseline
        self.last_label = None
        self.last_pad_time = None
        
        # Subscriptions
        self.pad_sub = self.create_subscription(
            PADState, "/kerfur/pad_state", self.on_pad, 10
        )
        
        # Publisher
        self.expr_pub = self.create_publisher(Expression, "/kerfur/expression", 10)
        
        # Timer for connection-loss detection and steady classification
        # 5Hz is plenty - the browser does its own animation
        self.timer = self.create_timer(0.2, self.tick)
        
        self.get_logger().info(
            f"PADToExpression started, hysteresis={self.hysteresis}, "
            f"timeout={self.timeout}s"
        )
    
    def on_pad(self, msg: PADState):
        self.current_pad = (msg.pleasure, msg.arousal, msg.dominance)
        if msg.mode:
            self.current_mode = msg.mode
        self.last_pad_time = self.get_clock().now()
    
    def tick(self):
        # Connection loss check - no PAD state received recently
        now = self.get_clock().now()
        if self.last_pad_time is None:
            # Haven't received anything yet
            self._publish_if_changed("error", 1.0)
            return
        
        age_sec = (now - self.last_pad_time).nanoseconds / 1e9
        if age_sec > self.timeout:
            self._publish_if_changed("error", 1.0)
            return
        
        # Classify current PAD into octant or neutral
        p, a, d = self.current_pad
        label, new_signs = classify_octant(p, a, d, self.hysteresis, self.prev_signs)
        self.prev_signs = new_signs
        
        # Intensity = distance from origin in PAD space, capped at 1.0
        intensity = min(1.0, (p*p + a*a + d*d) ** 0.5 / (3.0 ** 0.5))
        
        self._publish_if_changed(label, intensity)
    
    def _publish_if_changed(self, label, intensity):
        if label == self.last_label:
            return
        
        msg = Expression()
        msg.name = label
        msg.intensity = float(intensity)
        msg.duration_sec = 0.0
        self.expr_pub.publish(msg)
        
        p, a, d = self.current_pad
        self.get_logger().info(
            f"Expression: {self.last_label} -> {label} "
            f"(intensity {intensity:.2f}, PAD=[{p:+.2f},{a:+.2f},{d:+.2f}])"
        )
        self.last_label = label


def main():
    rclpy.init()
    node = PADToExpression()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
