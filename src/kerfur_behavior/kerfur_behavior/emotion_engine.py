"""Emotion engine: holds canonical PAD state, decays toward neutral, accepts nudges.

This is the single source of truth for Kerfur's affective state. It publishes
PADState continuously so downstream nodes always have a fresh value. Nudges from
reflexes or the cognition layer push the state around; decay pulls it back to
neutral over time.
"""

import rclpy
from rclpy.node import Node

from kerfur_msgs.msg import PADState, PADNudge


def clamp(value, lo=-1.0, hi=1.0):
    return max(lo, min(hi, value))


class EmotionEngine(Node):
    def __init__(self):
        super().__init__("emotion_engine")

        # Parameters
        self.declare_parameter("publish_rate_hz", 10.0)
        # Decay time constant: larger = moods last longer.
        # This is the time in seconds for a mood to decay to ~37% of its value.
        self.declare_parameter("decay_tau_sec", 8.0)
        # Default mode at startup
        self.declare_parameter("initial_mode", "default")

        self.publish_rate = self.get_parameter("publish_rate_hz").value
        self.decay_tau = self.get_parameter("decay_tau_sec").value
        self.mode = self.get_parameter("initial_mode").value

        # Canonical state - starts neutral
        self.pleasure = 0.0
        self.arousal = 0.0
        self.dominance = 0.0

        # Track time for decay calculation
        self.last_update = self.get_clock().now()

        # Subscriptions - nudges push the state
        self.nudge_sub = self.create_subscription(
            PADNudge, "/kerfur/pad_nudge", self.on_nudge, 10
        )

        # Publisher - continuous PAD state output
        self.pad_pub = self.create_publisher(PADState, "/kerfur/pad_state", 10)

        # Timer drives the publish + decay loop
        self.timer = self.create_timer(1.0 / self.publish_rate, self.tick)

        self.get_logger().info(
            f"EmotionEngine started: rate={self.publish_rate}Hz, "
            f"decay_tau={self.decay_tau}s, mode={self.mode}"
        )

    def on_nudge(self, msg: PADNudge):
        """Apply a state-dependent nudge: effect diminishes near the rails."""

        def saturate(current, nudge):
            if nudge > 0.0:
                headroom = 1.0 - current
            else:
                headroom = current + 1.0
            # Optional: clamp headroom to [0,1] for purely-diminishing behavior.
            # headroom = max(0.0, min(1.0, headroom))
            return clamp(current + nudge * headroom)

        self.pleasure = saturate(self.pleasure, msg.d_pleasure)
        self.arousal = saturate(self.arousal, msg.d_arousal)
        self.dominance = saturate(self.dominance, msg.d_dominance)

        if msg.mode_change:
            self.mode = msg.mode_change

        reason = msg.reason if msg.reason else "(no reason given)"
        self.get_logger().info(
            f"Nudge [{msg.d_pleasure:+.2f},{msg.d_arousal:+.2f},{msg.d_dominance:+.2f}] "
            f"-> PAD=[{self.pleasure:+.2f},{self.arousal:+.2f},{self.dominance:+.2f}] "
            f"reason: {reason}"
        )

    def tick(self):
        """Decay toward neutral and publish current state."""
        now = self.get_clock().now()
        dt = (now - self.last_update).nanoseconds / 1e9
        self.last_update = now

        # Exponential decay toward zero (neutral).
        # decay_factor approaches 0 as dt grows; for small dt it's ~1.
        # Using exp(-dt/tau) gives proper first-order decay regardless of tick rate.
        import math
        decay_factor = math.exp(-dt / self.decay_tau)
        self.pleasure *= decay_factor
        self.arousal *= decay_factor
        self.dominance *= decay_factor

        # Publish current state
        msg = PADState()
        msg.pleasure = float(self.pleasure)
        msg.arousal = float(self.arousal)
        msg.dominance = float(self.dominance)
        msg.mode = self.mode
        msg.stamp = now.to_msg()
        self.pad_pub.publish(msg)


def main():
    rclpy.init()
    node = EmotionEngine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
