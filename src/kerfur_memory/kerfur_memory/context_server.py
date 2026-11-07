"""Context server: assembles the prompt Kerfur's cognition nodes send to the LLM.

Single owner of prompt text and of the memory store. Cognition nodes hold
no prompt strings of their own - they call /kerfur/assemble_context and
forward what comes back.

Stage 2 serves four slots:
  identity  - who she is (static file, cacheable)
  voice     - output contract (static file, cacheable)
  core      - tier-1 memory rows (SQLite, cacheable, reloaded on a timer)
  snapshot  - live PAD state (volatile)

The response is split at the cache boundary: cacheable_prefix changes only
when a file or a core row changes; volatile_suffix changes every call.

Fails soft: a missing file, an unopenable store, or stale PAD sets
`degraded` rather than crashing.
"""

import hashlib
import os
import sqlite3
from dataclasses import dataclass

import yaml
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from kerfur_msgs.msg import PADState
from kerfur_msgs.srv import AssembleContext


CONFIG_PACKAGE = "kerfur_bringup"
CONFIG_SUBDIR = "config"
HASH_LEN = 12

# Memory tiers. 0 is reserved for identity should it ever move into the
# store; today identity is a file. 1 is always-in-prompt. 2 is archival,
# retrieved on demand (stage 4).
TIER_CORE = 1
TIER_ARCHIVAL = 2

SCHEMA = """
CREATE TABLE IF NOT EXISTS memory (
  id          INTEGER PRIMARY KEY,
  ts          INTEGER NOT NULL,
  kind        TEXT    NOT NULL,
  subject     TEXT,
  text        TEXT    NOT NULL,
  salience    REAL    NOT NULL,
  tier        INTEGER NOT NULL,
  last_seen   INTEGER NOT NULL
);
CREATE INDEX IF NOT EXISTS idx_memory_tier_sal ON memory(tier, salience DESC);
"""


@dataclass
class Slot:
    name: str
    body: str
    hash: str
    max_tokens: int
    ok: bool


def short_hash(text: str) -> str:
    return hashlib.sha256(text.encode("utf-8")).hexdigest()[:HASH_LEN]


def count_tokens(text: str) -> int:
    return len(text.split())


def split_frontmatter(raw: str):
    stripped = raw.lstrip()
    if not stripped.startswith("---"):
        return {}, raw
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
        self.declare_parameter("pad_stale_sec", 2.0)
        self.declare_parameter("log_full_prompt", True)
        # Store location. Outside the workspace so a rebuild never touches it.
        self.declare_parameter("memory_db_path", os.path.expanduser("~/.kerfur/memory.db"))
        # Hard cap on the core slot. Lowest-salience rows fall off first.
        self.declare_parameter("core_max_tokens", 500)
        # How often core is re-read from disk. Core changes rarely; per-call
        # reads would be wasted I/O on the Pi.
        self.declare_parameter("core_reload_sec", 30.0)

        self.pad_stale_sec = self.get_parameter("pad_stale_sec").value
        self.log_full_prompt = self.get_parameter("log_full_prompt").value
        self.db_path = self.get_parameter("memory_db_path").value
        self.core_max_tokens = self.get_parameter("core_max_tokens").value
        self.core_reload_sec = self.get_parameter("core_reload_sec").value

        # --- Static slots ---
        config_dir = os.path.join(
            get_package_share_directory(CONFIG_PACKAGE), CONFIG_SUBDIR
        )
        self.identity = self._load_slot("identity", config_dir, default_max=250)
        self.voice = self._load_slot("voice", config_dir, default_max=150)

        # --- Store ---
        self.db = self._open_store()
        self.core = self._load_core()

        self.get_logger().info(
            f"Slots loaded: identity={self.identity.hash} (ok={self.identity.ok}) "
            f"voice={self.voice.hash} (ok={self.voice.ok}) "
            f"core={self.core.hash} (ok={self.core.ok}, {count_tokens(self.core.body)} tok)"
        )

        # --- Live state ---
        self.last_pad = None
        self.last_pad_time = None
        self.create_subscription(PADState, "/kerfur/pad_state", self.on_pad, 10)

        # --- Timers ---
        self.create_timer(self.core_reload_sec, self._reload_core)

        # --- Service ---
        self.create_service(
            AssembleContext, "/kerfur/assemble_context", self.on_assemble
        )
        self.get_logger().info("ContextServer ready on /kerfur/assemble_context")

    # ------------------------------------------------------------------
    # Static slots
    # ------------------------------------------------------------------

    def _load_slot(self, name: str, config_dir: str, default_max: int) -> Slot:
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
        return Slot(name=name, body=body, hash=short_hash(body),
                    max_tokens=max_tokens, ok=True)

    # ------------------------------------------------------------------
    # Store
    # ------------------------------------------------------------------

    def _open_store(self):
        """Open (or create) the SQLite store. Returns None on failure -
        the node runs without core memory rather than dying."""
        try:
            os.makedirs(os.path.dirname(self.db_path), exist_ok=True)
            # check_same_thread=False: the timer and the service callback
            # may run on different executor threads later. Access is
            # read-only from this node for now, so no locking needed yet.
            db = sqlite3.connect(self.db_path, check_same_thread=False)
            db.execute("PRAGMA journal_mode=WAL")
            db.executescript(SCHEMA)
            db.commit()
            self.get_logger().info(f"Memory store open: {self.db_path}")
            return db
        except sqlite3.Error as e:
            self.get_logger().error(f"Memory store unavailable: {e}")
            return None

    def _load_core(self) -> Slot:
        """Read tier-1 rows into the core slot, highest salience first,
        truncated at core_max_tokens by whole rows. A row that doesn't fit
        is dropped, never cut mid-sentence."""
        if self.db is None:
            return Slot(name="core", body="", hash="nostore",
                        max_tokens=self.core_max_tokens, ok=False)
        try:
            rows = self.db.execute(
                "SELECT text FROM memory WHERE tier = ? "
                "ORDER BY salience DESC, last_seen DESC",
                (TIER_CORE,),
            ).fetchall()
        except sqlite3.Error as e:
            self.get_logger().error(f"Core read failed: {e}")
            return Slot(name="core", body="", hash="readfail",
                        max_tokens=self.core_max_tokens, ok=False)

        header = "What Kerfur remembers:"
        lines, used, dropped = [header], count_tokens(header), 0
        for (text,) in rows:
            t = count_tokens(text)
            if used + t > self.core_max_tokens:
                dropped += 1
                continue
            lines.append(f"- {text}")
            used += t
        if dropped:
            self.get_logger().warn(
                f"Core truncated: {dropped} row(s) over {self.core_max_tokens} tokens"
            )

        # No rows at all: an empty core is valid, not degraded. She just
        # doesn't remember anything yet.
        body = "\n".join(lines) if len(lines) > 1 else ""
        return Slot(name="core", body=body, hash=short_hash(body),
                    max_tokens=self.core_max_tokens, ok=(dropped == 0))

    def _reload_core(self):
        new = self._load_core()
        if new.hash != self.core.hash:
            self.get_logger().info(
                f"Core changed: {self.core.hash} -> {new.hash} "
                f"({count_tokens(new.body)} tok)"
            )
        self.core = new

    # ------------------------------------------------------------------
    # Live state
    # ------------------------------------------------------------------

    def on_pad(self, msg: PADState):
        self.last_pad = msg
        self.last_pad_time = self.get_clock().now()

    def _pad_is_stale(self) -> bool:
        if self.last_pad_time is None:
            return True
        age = (self.get_clock().now() - self.last_pad_time).nanoseconds / 1e9
        return age > self.pad_stale_sec

    def _build_snapshot(self, stale: bool) -> str:
        if stale:
            return "Current state:\nstate unavailable"
        p = self.last_pad
        return (
            "Current state:\n"
            f"pleasure={p.pleasure:+.2f} arousal={p.arousal:+.2f} "
            f"dominance={p.dominance:+.2f} mode={p.mode}"
        )

    # ------------------------------------------------------------------
    # Assembly
    # ------------------------------------------------------------------

    def _cap(self, text: str, max_tokens: int, slot_name: str):
        words = text.split()
        if len(words) <= max_tokens:
            return text, False
        self.get_logger().warn(
            f"Slot '{slot_name}' truncated: {len(words)} > {max_tokens} tokens"
        )
        return " ".join(words[:max_tokens]), True

    def on_assemble(self, request, response):
        identity_body, t1 = self._cap(self.identity.body, self.identity.max_tokens, "identity")
        voice_body, t2 = self._cap(self.voice.body, self.voice.max_tokens, "voice")
        truncated = t1 or t2 or not self.core.ok

        # --- Cacheable prefix: identity, voice, core. Static per reload. ---
        prefix_parts = [identity_body, voice_body]
        if self.core.body:
            prefix_parts.append(self.core.body)
        response.cacheable_prefix = "\n\n".join(prefix_parts)

        # --- Volatile suffix ---
        pad_stale = self._pad_is_stale()
        snapshot = self._build_snapshot(pad_stale)
        suffix_parts = [snapshot]
        if request.agent_result:
            suffix_parts.append("Facts:\n" + request.agent_result)
        suffix_parts.append(request.trigger_text)
        response.volatile_suffix = "\n\n".join(suffix_parts)

        # --- Bookkeeping: four slots now ---
        response.slot_hashes = [
            self.identity.hash,
            self.voice.hash,
            self.core.hash,
            short_hash(snapshot),
        ]
        response.slot_tokens = [
            count_tokens(identity_body),
            count_tokens(voice_body),
            count_tokens(self.core.body),
            count_tokens(snapshot),
        ]
        response.degraded = (
            not self.identity.ok
            or not self.voice.ok
            or pad_stale
            or truncated
        )

        self.get_logger().info(
            f"Assembled [{request.pass_type}] hashes="
            f"{'/'.join(response.slot_hashes)} "
            f"tokens={list(response.slot_tokens)} degraded={response.degraded}"
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
        if node.db is not None:
            node.db.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
