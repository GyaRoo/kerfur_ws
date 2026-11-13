"""Consciousness layer: rare, deliberate lucidity for the familiar.

Wakes on a trigger, asks the context server for the assembled prompt, sends
it through the LiteLLM gateway, and publishes an utterance plus a mood nudge.
Holds no prompt text and no provider knowledge of its own. Fails soft: if
the context server or the gateway is unreachable, Kerfur stays an animal.
"""

import os
import json
import threading
import urllib.request
import urllib.error

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from kerfur_msgs.msg import ConsciousnessTrigger, Utterance, PADNudge, PADState
from kerfur_msgs.srv import AssembleContext


KEY_NAME = "LITELLM_MASTER_KEY"

# Output contract. Appended to the volatile half of the prompt so the model
# knows the response shape. Kept here rather than in voice.md because it's
# plumbing, not character - voice.md constrains what she says, this
# constrains how the node parses it.
FORMAT_INSTRUCTION = """Respond ONLY with a JSON object, no other text:
{
  "thinking": "brief private reasoning, not spoken",
  "utterance": "what Kerfur actually says",
  "pleasure": 0.0,
  "arousal": 0.0,
  "dominance": 0.0
}
pleasure/arousal/dominance are small nudges (deltas, roughly -0.4 to +0.4)
describing how this moment shifts her mood."""


class ConsciousnessNode(Node):
    def __init__(self):
        super().__init__("consciousness")

        # --- Parameters ---
        # Gateway endpoint and the alias configured in LiteLLM's config.yaml.
        # The node never sees a provider name; swapping backends is a
        # gateway config change.
        self.declare_parameter("llm_url", "http://llm.kerfur.lan:8400/v1/chat/completions")
        self.declare_parameter("llm_model", "kerfur-consciousness")
        self.declare_parameter("llm_timeout_sec", 30.0)
        self.declare_parameter("context_timeout_sec", 2.0)
        # Path to the secrets file, never the key itself: parameters are
        # readable by anything on the graph via `ros2 param get`.
        self.declare_parameter("secrets_file", "~/.kerfur/secrets.env")

        self.llm_url = self.get_parameter("llm_url").value
        self.llm_model = self.get_parameter("llm_model").value
        self.llm_timeout = self.get_parameter("llm_timeout_sec").value
        self.context_timeout = self.get_parameter("context_timeout_sec").value

        # Gateway key. Environment wins if present; otherwise the node reads
        # the secrets file itself, so no launch path can forget the export.
        self.api_key = self._load_key()
        if not self.api_key:
            self.get_logger().warn(
                f"No {KEY_NAME} in environment or secrets file - "
                "will retry on first wake."
            )

        # Current mood, kept for the Utterance snapshot. The prompt's copy
        # of PAD comes from the context server, not from here.
        self.current_pad = (0.0, 0.0, 0.0)

        # Reentrant group so the blocking work in on_wake (service call +
        # HTTP) doesn't stall on_pad or anything else on this node.
        self.cb_group = ReentrantCallbackGroup()

        # --- Context service client ---
        self.context_client = self.create_client(
            AssembleContext, "/kerfur/assemble_context",
            callback_group=self.cb_group,
        )

        # --- Subscriptions ---
        self.create_subscription(
            ConsciousnessTrigger, "/kerfur/wake", self.on_wake, 10,
            callback_group=self.cb_group,
        )
        self.create_subscription(
            PADState, "/kerfur/pad_state", self.on_pad, 10,
        )

        # --- Publishers ---
        self.utterance_pub = self.create_publisher(Utterance, "/kerfur/utterance", 10)
        self.nudge_pub = self.create_publisher(PADNudge, "/kerfur/pad_nudge", 10)

        self.get_logger().info(
            f"Consciousness online: model={self.llm_model} via {self.llm_url}"
        )

    def on_pad(self, msg: PADState):
        self.current_pad = (msg.pleasure, msg.arousal, msg.dominance)

    # ------------------------------------------------------------------
    # Key
    # ------------------------------------------------------------------

    def _load_key(self):
        """Gateway key: environment first, then the secrets file.

        Accepts both `KEY=value` and `export KEY=value` lines, with or
        without quotes. Returns None if neither source has it.
        """
        key = os.environ.get(KEY_NAME)
        if key:
            return key

        path = os.path.expanduser(self.get_parameter("secrets_file").value)
        try:
            with open(path, "r", encoding="utf-8") as f:
                for line in f:
                    line = line.strip()
                    if line.startswith("export "):
                        line = line[len("export "):].lstrip()
                    if line.startswith(KEY_NAME + "="):
                        value = line.split("=", 1)[1].strip().strip("'\"")
                        return value or None
        except OSError as e:
            self.get_logger().warn(f"Secrets file unreadable: {e}")
        return None

    # ------------------------------------------------------------------
    # Context
    # ------------------------------------------------------------------

    def _assemble_context(self, trigger_type: str, content: str):
        """Ask the context server for the prompt. Returns the response or
        None if the server is absent or slow."""
        if not self.context_client.wait_for_service(timeout_sec=self.context_timeout):
            self.get_logger().warn("Context server unavailable.")
            return None

        req = AssembleContext.Request()
        req.pass_type = "render"
        req.trigger_text = f"What woke you: [{trigger_type}] {content}"
        req.agent_result = ""

        # Wait on an Event rather than spinning. This callback already runs
        # on the MultiThreadedExecutor; the client is in a reentrant group,
        # so another executor thread delivers the response and fires the
        # done-callback while this thread blocks. Spinning the node again
        # from here would put it on a second executor.
        done = threading.Event()
        future = self.context_client.call_async(req)
        future.add_done_callback(lambda _: done.set())

        if not done.wait(timeout=self.context_timeout):
            self.context_client.remove_pending_request(future)
            self.get_logger().warn("Context assembly timed out.")
            return None
        if future.result() is None:
            self.get_logger().warn("Context assembly returned nothing.")
            return None
        return future.result()

    # ------------------------------------------------------------------
    # Gateway
    # ------------------------------------------------------------------

    def _call_gateway(self, system_text: str, user_text: str) -> dict:
        """OpenAI-compatible chat completion via LiteLLM. Raises on any
        transport or parse failure; the caller handles fail-soft."""
        # Late retry: a key added to the secrets file after startup is
        # picked up on the next wake without restarting the node.
        if not self.api_key:
            self.api_key = self._load_key()
        if not self.api_key:
            raise RuntimeError(f"no {KEY_NAME} available")

        payload = {
            "model": self.llm_model,
            "max_tokens": 300,
            "messages": [
                # Prefix as system message: static, cacheable.
                {"role": "system", "content": system_text},
                # Suffix as user message: per-call.
                {"role": "user", "content": user_text},
            ],
            "response_format": {"type": "json_object"},
        }
        data = json.dumps(payload).encode("utf-8")
        req = urllib.request.Request(
            self.llm_url, data=data,
            headers={
                "Content-Type": "application/json",
                "Authorization": f"Bearer {self.api_key}",
            },
        )
        with urllib.request.urlopen(req, timeout=self.llm_timeout) as resp:
            body = json.loads(resp.read().decode("utf-8"))

        text = body["choices"][0]["message"]["content"].strip()
        # Defensive: some models fence JSON even when told not to.
        if text.startswith("```"):
            text = text.strip("`")
            if text.startswith("json"):
                text = text[4:]
        return json.loads(text)

    # ------------------------------------------------------------------
    # Wake handling
    # ------------------------------------------------------------------

    def on_wake(self, msg: ConsciousnessTrigger):
        self.get_logger().info(
            f"Woken [{msg.trigger_type}] from {msg.source}: {msg.content}"
        )

        ctx = self._assemble_context(msg.trigger_type, msg.content)
        if ctx is None:
            self.get_logger().warn("No context - staying a creature.")
            return
        if ctx.degraded:
            # Proceed, but say so. A degraded prompt is still a prompt;
            # the log tells you why the answer might be off.
            self.get_logger().warn(
                f"Context degraded (hashes={'/'.join(ctx.slot_hashes)})"
            )

        user_text = ctx.volatile_suffix + "\n\n" + FORMAT_INSTRUCTION

        try:
            result = self._call_gateway(ctx.cacheable_prefix, user_text)
        except Exception as e:
            self.get_logger().warn(f"Gateway call failed ({e}) - staying a creature.")
            return

        utterance_text = result.get("utterance", "")
        thinking = result.get("thinking", "")

        utt = Utterance()
        utt.text = utterance_text
        utt.kind = "lucid"
        utt.pleasure = float(self.current_pad[0])
        utt.arousal = float(self.current_pad[1])
        utt.dominance = float(self.current_pad[2])
        utt.stamp = self.get_clock().now().to_msg()
        self.utterance_pub.publish(utt)

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
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
