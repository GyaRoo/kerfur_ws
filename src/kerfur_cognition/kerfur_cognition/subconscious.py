"""Subconscious cognition node: takes situations, produces PAD nudges via an LLM.

This is the 'fast intuition' layer. It reacts to situations with affective nudges
that push the emotion engine's PAD state. It does NOT set mood directly - it
expresses 'this situation makes you somewhat more X' and lets the engine integrate.

Backend is pluggable: 'mock' for offline dev, 'anthropic' for the real API,
'ollama' for a local model. Same interface throughout.
"""

import json
import random

import rclpy
from rclpy.node import Node

from kerfur_msgs.msg import Situation, PADNudge


# The contract we ask the LLM to fulfill. Kept here so mock and real backends
# produce the same shape.
SYSTEM_PROMPT = """You are the affective subconscious of Kerfur, a small companion robot \
with an animal-like temperament (think of a cat with a synthesizer voice). You do NOT think \
in words or make plans. You produce gut-level emotional reactions to situations.

Given a described situation, respond with how it makes Kerfur feel, as a small nudge to its \
emotional state along three axes:
- pleasure: -1 (unpleasant) to +1 (pleasant)
- arousal: -1 (calm) to +1 (excited)
- dominance: -1 (submissive/uncertain) to +1 (in-control/assertive)

These are NUDGES (deltas), not absolute states. Keep each value small, roughly between \
-0.5 and +0.5. A mild reaction might be 0.1-0.2; a strong one 0.4-0.5. The nudges accumulate \
in a separate system and decay over time, so you don't need large values.

Respond ONLY with a JSON object, no other text, in exactly this form:
{"reasoning": "one short phrase", "pleasure": 0.0, "arousal": 0.0, "dominance": 0.0, "mode_change": null}

mode_change is almost always null. Only suggest a mode ("sleepy", "playful", "alert", "sick", \
"bonding") if the situation strongly calls for a sustained shift."""


class MockBackend:
    """Produces plausible nudges from keyword heuristics. No API needed."""

    def react(self, situation: str) -> dict:
        s = situation.lower()
        p = a = d = 0.0
        reason = "neutral"

        # Crude keyword reactions - enough to test the pipeline
        if any(w in s for w in ["pet", "stroke", "pat", "gentle", "soft"]):
            p, a, d, reason = 0.4, -0.1, 0.1, "pleasant touch"
        elif any(w in s for w in ["loud", "bang", "crash", "sudden", "startle"]):
            p, a, d, reason = -0.3, 0.5, -0.4, "startled"
        elif any(w in s for w in ["approach", "person", "someone", "visitor"]):
            p, a, d, reason = 0.2, 0.3, 0.0, "curious about approach"
        elif any(w in s for w in ["alone", "empty", "quiet", "nobody"]):
            p, a, d, reason = -0.1, -0.3, -0.1, "lonely quiet"
        elif any(w in s for w in ["play", "toy", "game", "chase"]):
            p, a, d, reason = 0.4, 0.4, 0.2, "playful"
        elif any(w in s for w in ["food", "treat", "feed"]):
            p, a, d, reason = 0.5, 0.3, 0.1, "treat incoming"
        else:
            # Mild random drift so it's not totally flat on unknown input
            p = random.uniform(-0.1, 0.1)
            a = random.uniform(-0.1, 0.1)
            reason = "mild ambient reaction"

        return {
            "reasoning": reason,
            "pleasure": p, "arousal": a, "dominance": d,
            "mode_change": None,
        }


class SubconsciousNode(Node):
    def __init__(self):
        super().__init__("subconscious")

        self.declare_parameter("backend", "mock")  # mock | anthropic | ollama
        self.backend_name = self.get_parameter("backend").value

        if self.backend_name == "mock":
            self.backend = MockBackend()
        else:
            # Real backends wired in later - fall back to mock for now
            self.get_logger().warn(
                f"Backend '{self.backend_name}' not yet implemented, using mock"
            )
            self.backend = MockBackend()

        self.sub = self.create_subscription(
            Situation, "/kerfur/situation", self.on_situation, 10
        )
        self.nudge_pub = self.create_publisher(PADNudge, "/kerfur/pad_nudge", 10)

        self.get_logger().info(f"Subconscious started, backend={self.backend_name}")

    def on_situation(self, msg: Situation):
        self.get_logger().info(f"Situation: {msg.description}")
        try:
            result = self.backend.react(msg.description)
        except Exception as e:
            self.get_logger().error(f"Backend failed: {e}")
            return

        nudge = PADNudge()
        nudge.d_pleasure = float(result.get("pleasure", 0.0))
        nudge.d_arousal = float(result.get("arousal", 0.0))
        nudge.d_dominance = float(result.get("dominance", 0.0))
        nudge.mode_change = result.get("mode_change") or ""
        nudge.reason = result.get("reasoning", "")
        self.nudge_pub.publish(nudge)

        self.get_logger().info(
            f"Nudge [{nudge.d_pleasure:+.2f},{nudge.d_arousal:+.2f},"
            f"{nudge.d_dominance:+.2f}] reason: {nudge.reason}"
        )


def main():
    rclpy.init()
    node = SubconsciousNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
