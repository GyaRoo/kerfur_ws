"""Context server: assembles the prompt Kerfur's cognition nodes send to the LLM.

Single owner of prompt text. Cognition nodes hold no prompt strings of their
own - they call /kerfur/assemble_context and forward what comes back.

Stage 1 serves three slots:
  identity  - who she is (static file, cacheable)
  voice     - output contract (static file, cacheable)
  snapshot  - live PAD state (volatile)

The response is split at the cache boundary: cacheable_prefix never contains
anything that changes between calls; volatile_suffix carries everything that
does. Interleaving them would silently defeat prompt caching upstream.

Fails soft: a missing file or stale PAD sets `degraded` rather than crashing.
"""

import hashlib
import os
from dataclasses import dataclass

import yaml
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from kerfur_msgs.msg import PADState
from kerfur_msgs.srv import AssembleContext


# Where the static prompt files live. They are installed alongside
# kerfur_params.yaml by the bringup package's setup.py glob.
CONFIG_PACKAGE = "kerfur_bringup"
CONFIG_SUBDIR = "config"

# Hash length. 12 hex chars is plenty to disambiguate and keeps log lines short.
HASH_LEN = 12


@dataclass
class Slot:
    """One prompt slot: a body of text plus the metadata the server tracks."""
    name: str
    body: str
    hash: str
    max_tokens: int
    ok: bool


def short_hash(text: str) -> str:
    """Stable fingerprint of a text body. Logged with every prompt so a change
    in her behavior can be tied to a change in what she was told."""
    return hashlib.sha256(text.encode("utf-8")).hexdigest()[:HASH_LEN]


def count_tokens(text: str) -> int:
    """Rough token count. Word count is a crude proxy but it's consistent,
    which is what matters for a cap. Swap for a real tokenizer later if the
    cap ever needs to be precise."""
    return len(text.split())


def split_frontmatter(raw: str):
    """Split a markdown file into (meta_dict, body).

    Expected layout:
        ---
        version: 1
        max_tokens: 250
        ---
        body text...

    If there's no frontmatter, meta is empty and the whole file is the body.
    """
    stripped = raw.lstrip()
    if not stripped.startswith("---"):
        return {}, raw

    # Split on the first two '---' markers. parts[0] is empty (before the
    # first marker), parts[1] is the YAML, parts[2] is the body.
    parts = stripped.split("---", 2)
    if len(parts) < 3:
        return {}, raw

    meta = yaml.safe_load(parts[1]) or {}
    body = parts[2].strip()
    return meta, body


class ContextServer(Node):
    def __init__(self):
        super().__init__("context_server")

        # --- Parameters ---
        # PAD older than this is considered stale and the snapshot reports
        # "state unavailable" rather than old numbers.
        self.declare_parameter("pad_stale_sec", 2.0)
        # Log the full assembled prompt at INFO. Turn off once it's noisy.
        self.declare_parameter("log_full_prompt", True)

        self.pad_stale_sec = self.get_parameter("pad_stale_sec").value
        self.log_full_prompt = self.get_parameter("log_full_prompt").value

        # --- Static slots, loaded once ---
        config_dir = os.path.join(
            get_package_share_directory(CONFIG_PACKAGE), CONFIG_SUBDIR
        )
        self.identity = self._load_slot("identity", config_dir, default_max=250)
        self.voice = self._load_slot("voice", config_dir, default_max=150)

        # Log hashes once at startup. This is the line you grep for when
        # correlating a behavior shift to a file edit.
        self.get_logger().info(
            f"Slots loaded: identity={self.identity.hash} "
            f"(ok={self.identity.ok}) voice={self.voice.hash} "
            f"(ok={self.voice.ok})"
        )

        # --- Live state ---
        self.last_pad = None
        self.last_pad_time = None

        self.create_subscription(PADState, "/kerfur/pad_state", self.on_pad, 10)

        # --- Service ---
        self.create_service(
            AssembleContext, "/kerfur/assemble_context", self.on_assemble
        )

        self.get_logger().info("ContextServer ready on /kerfur/assemble_context")

    # ------------------------------------------------------------------
    # Loading
    # ------------------------------------------------------------------

    def _load_slot(self, name: str, config_dir: str, default_max: int) -> Slot:
        """Read <name>.md from the config dir into a Slot. Never raises -
        a missing or malformed file yields an empty slot with ok=False."""
        path = os.path.join(config_dir, f"{name}.md")
        try:
            with open(path, "r", encoding="utf-8") as f:
                raw = f.read()
        except (FileNotFoundError, OSError) as e:
            self.get_logger().error(f"Slot '{name}' unavailable: {e}")
            return Slot(name=name, body="", hash="missing",
                        max_tokens=default_max, ok=False)

        meta, body = split_frontmatter(raw)
        max_tokens = int(meta.get("max_tokens", default_max))

        # Hash the body only. Bumping `version` in frontmatter shouldn't
        # change the hash - the hash tracks what the model actually sees.
        return Slot(name=name, body=body, hash=short_hash(body),
                    max_tokens=max_tokens, ok=True)

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------

    def on_pad(self, msg: PADState):
        self.last_pad = msg
        self.last_pad_time = self.get_clock().now()

    # ------------------------------------------------------------------
    # Assembly
    # ------------------------------------------------------------------

    def _pad_is_stale(self) -> bool:
        if self.last_pad_time is None:
            return True
        age = (self.get_clock().now() - self.last_pad_time).nanoseconds / 1e9
        return age > self.pad_stale_sec

    def _build_snapshot(self) -> str:
        """Live state block. Uses PADState field names verbatim so log greps
        and the model see the same vocabulary."""
        if self._pad_is_stale():
            # Tell the model it's blind rather than inventing zeros.
            return "Current state:\nstate unavailable"

        p = self.last_pad
        return (
            "Current state:\n"
            f"pleasure={p.pleasure:+.2f} arousal={p.arousal:+.2f} "
            f"dominance={p.dominance:+.2f} mode={p.mode}"
        )

    def _cap(self, text: str, max_tokens: int, slot_name: str):
        """Truncate to max_tokens words. Returns (text, was_truncated).
        Truncation is always logged - a silently shortened identity is
        worse than a loud one."""
        words = text.split()
        if len(words) <= max_tokens:
            return text, False
        self.get_logger().warn(
            f"Slot '{slot_name}' truncated: {len(words)} > {max_tokens} tokens"
        )
        return " ".join(words[:max_tokens]), True

    def on_assemble(self, request, response):
        truncated = False

        # --- Cacheable prefix: static slots only ---
        identity_body, t1 = self._cap(
            self.identity.body, self.identity.max_tokens, "identity"
        )
        voice_body, t2 = self._cap(
            self.voice.body, self.voice.max_tokens, "voice"
        )
        truncated = truncated or t1 or t2

        # Nothing dynamic may ever be added to this string. It is the cache
        # key for everything upstream.
        response.cacheable_prefix = identity_body + "\n\n" + voice_body

        # --- Volatile suffix: snapshot + trigger ---
        snapshot = self._build_snapshot()
        pad_stale = self._pad_is_stale()

        # Stage 1: agent and render passes get the same suffix. The split
        # becomes real when tools and the memory store arrive.
        suffix_parts = [snapshot]
        if request.agent_result:
            suffix_parts.append("Facts:\n" + request.agent_result)
        suffix_parts.append(request.trigger_text)
        response.volatile_suffix = "\n\n".join(suffix_parts)

        # --- Bookkeeping ---
        response.slot_hashes = [
            self.identity.hash,
            self.voice.hash,
            short_hash(snapshot),
        ]
        response.slot_tokens = [
            count_tokens(identity_body),
            count_tokens(voice_body),
            count_tokens(snapshot),
        ]
        response.degraded = (
            not self.identity.ok
            or not self.voice.ok
            or pad_stale
            or truncated
        )

        # --- The deliverable: a verbatim record of what she was told ---
        self.get_logger().info(
            f"Assembled [{request.pass_type}] hashes="
            f"{'/'.join(response.slot_hashes)} "
            f"tokens={response.slot_tokens} degraded={response.degraded}"
        )
        if self.log_full_prompt:
            self.get_logger().info(
                "--- PREFIX ---\n" + response.cacheable_prefix +
                "\n--- SUFFIX ---\n" + response.volatile_suffix +
                "\n--- END ---"
            )

        return response


def main():
    rclpy.init()
    node = ContextServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()