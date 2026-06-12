"""Consciousness layer: rare, deliberate lucidity for the familiar.

Wakes on a trigger (direct address now; HA events / novelty later), considers
the situation IN THE CREATURE'S CURRENT MOOD via a cloud model, and produces
an utterance + optional mood nudge. Surfaces through the creature - it does not
bypass the affective system. Fails soft: if the cloud is unreachable or no key
is present, the familiar simply stays an animal.
"""

import os
import json
import urllib.request
import urllib.error

import rclpy
from rclpy.node import Node

from kerfur_msgs.msg import ConsciousnessTrigger, Utterance, PADNudge, PADState


SYSTEM_PROMPT = """You are the conscious, lucid mind of Kerfur - a small companion \
robot that is fundamentally an anthropomorphic animal (feline coded) and capable of \
rising to genuine, articulate thought when addressed or when something remarkable \
happens.

You are given: what woke you (an address, an event, or something novel), and Kerfur's \
CURRENT EMOTIONAL STATE as PAD values (pleasure, arousal, dominance, each -1 to +1). \
Your response must be COLORED BY THIS MOOD - a lucid Kerfur that is anxious speaks \
differently than one that is joyful or relaxed. You are still the creature; you are \
just briefly able to articulate.

Respond with what Kerfur should say, and how this moment of lucidity affects its mood.

Respond ONLY with a JSON object, no other text:
{
  "thinking": "brief private reasoning, not spoken",
  "utterance": "what Kerfur actually says, in character - a creature briefly lucid, not a chatbot",
  "pleasure": 0.0,
  "arousal": 0.0,
  "dominance": 0.0
}

The pleasure/arousal/dominance are small NUDGES (deltas, roughly -0.4 to +0.4) - how \
this moment of thought shifts Kerfur's mood. Keep the utterance short and creaturely; \
Kerfur is a familiar, not an assistant reading a manual."""


class AnthropicBackend:
    """Calls the Anthropic API for consciousness. Swappable - same interface a
    future local/self-hosted backend would implement."""

    def __init__(self, api_key, model, logger):
        self.api_key = api_key
        self.model = model
        self.logger = logger
        self.url = "https://api.anthropic.com/v1/messages"

    def deliberate(self, trigger_type, content, pad):
        user_msg = (
            f"What woke you: [{trigger_type}] {content}\n\n"
            f"Kerfur's current mood (PAD): pleasure={pad[0]:.2f}, "
            f"arousal={pad[1]:.2f}, dominance={pad[2]:.2f}\n\n"
            f"Respond with the JSON object only."
        )
        payload = {
            "model": self.model,
            "max_tokens": 300,
            "system": SYSTEM_PROMPT,
            "messages": [{"role": "user", "content": user_msg}],
        }
        data = json.dumps(payload).encode("utf-8")
        req = urllib.request.Request(
            self.url, data=data,
            headers={
                "Content-Type": "application/json",
                "x-api-key": self.api_key,
                "anthropic-version": "2023-06-01",
            },
        )
        with urllib.request.urlopen(req, timeout=30) as resp:
            body = json.loads(resp.read().decode("utf-8"))
        # Anthropic returns content as a list of blocks; take the text.
        text = "".join(
            block.get("text", "")
            for block in body.get("content", [])
            if block.get("type") == "text"
        )
        # Strip any stray formatting and parse the JSON the model returned.
        text = text.strip()
        if text.startswith("```"):
            text = text.strip("`")
            if text.startswith("json"):
                text = text[4:]
        return json.loads(text)


class ConsciousnessNode(Node):
    def __init__(self):
        super().__init__("consciousness")

        self.declare_parameter("model", "claude-opus-4-8")
        self.model = self.get_parameter("model").value

        # Current creature mood - updated continuously, used to color lucidity
        self.current_pad = (0.0, 0.0, 0.0)

        # Backend - fail soft if no key
        api_key = os.environ.get("ANTHROPIC_API_KEY")
        if api_key:
            self.backend = AnthropicBackend(api_key, self.model, self.get_logger())
            self.get_logger().info(f"Consciousness online, model={self.model}")
        else:
            self.backend = None
            self.get_logger().warn(
                "No ANTHROPIC_API_KEY - consciousness unavailable, "
                "Kerfur stays a creature."
            )

        # Subscriptions
        self.create_subscription(
            ConsciousnessTrigger, "/kerfur/wake", self.on_wake, 10
        )
        self.create_subscription(
            PADState, "/kerfur/pad_state", self.on_pad, 10
        )

        # Publishers
        self.utterance_pub = self.create_publisher(Utterance, "/kerfur/utterance", 10)
        self.nudge_pub = self.create_publisher(PADNudge, "/kerfur/pad_nudge", 10)
        # Lucidity signal - tells the behavior layer Kerfur is "thinking"
        self.lucid_pub = self.create_publisher(Expression, "/kerfur/expression", 10) \
            if False else None  # placeholder - see note below

    def on_pad(self, msg: PADState):
        self.current_pad = (msg.pleasure, msg.arousal, msg.dominance)

    def on_wake(self, msg: ConsciousnessTrigger):
        self.get_logger().info(
            f"Woken [{msg.trigger_type}] from {msg.source}: {msg.content}"
        )

        if self.backend is None:
            self.get_logger().warn("Consciousness unavailable - no lucidity this time.")
            return

        try:
            result = self.backend.deliberate(
                msg.trigger_type, msg.content, self.current_pad
            )
        except Exception as e:
            # Fail soft - the familiar just doesn't rise to lucidity right now
            self.get_logger().warn(f"Consciousness call failed ({e}) - staying a creature.")
            return

        utterance_text = result.get("utterance", "")
        thinking = result.get("thinking", "")

        # Publish the utterance (awaiting the voice layer; logged for now)
        utt = Utterance()
        utt.text = utterance_text
        utt.kind = "lucid"
        utt.pleasure = float(self.current_pad[0])
        utt.arousal = float(self.current_pad[1])
        utt.dominance = float(self.current_pad[2])
        utt.stamp = self.get_clock().now().to_msg()
        self.utterance_pub.publish(utt)

        # Publish the mood nudge through the normal engine
        nudge = PADNudge()
        nudge.d_pleasure = float(result.get("pleasure", 0.0))
        nudge.d_arousal = float(result.get("arousal", 0.0))
        nudge.d_dominance = float(result.get("dominance", 0.0))
        nudge.mode_change = ""
        nudge.reason = "consciousness (lucid)"
        self.nudge_pub.publish(nudge)

        self.get_logger().info(f"[thinking: {thinking}]")
        self.get_logger().info(f'Kerfur says: "{utterance_text}"')


def main():
    rclpy.init()
    node = ConsciousnessNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
