"""Voice subsystem: turns utterances into blabber scores.

Subscribes to /kerfur/utterance, builds a per-syllable score (pitch,
duration, gap) colored by the PAD snapshot the utterance carries, and
publishes it as a Speech message. Nothing is synthesized here - the face
browser plays the score through Web Audio and reveals the text in step.

Python decides what she sounds like; the browser only plays it. Same
split as expressions. Every coefficient is a parameter so the voice is
tuned from kerfur_params.yaml, not from code.
"""

import rclpy
from rclpy.node import Node

from kerfur_msgs.msg import Utterance, Speech


VOWELS = set("aeiouy")


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def syllabify(text: str):
    """Split text into syllable-ish chunks, one blip each.

    Vowel-group counting: a new chunk starts at each vowel run after the
    current chunk already holds one. Wrong on a fair share of English
    words and it doesn't matter - this is the rhythm of speech, not
    speech. Each chunk keeps its trailing space (except the last) so the
    overlay can concatenate them straight back into the original line.
    """
    out = []
    for word in text.split(" "):
        if not word:
            continue
        chunks, cur, in_vowel = [], "", False
        for ch in word:
            is_v = ch.lower() in VOWELS
            if is_v and not in_vowel and any(c.lower() in VOWELS for c in cur):
                chunks.append(cur)
                cur = ""
            cur += ch
            in_vowel = is_v
        if cur:
            chunks.append(cur)
        chunks[-1] += " "
        out.extend(chunks)
    if out:
        out[-1] = out[-1].rstrip()
    return out


class VoiceNode(Node):
    def __init__(self):
        super().__init__("voice")

        # --- Base voice ---
        self.declare_parameter("base_hz", 440.0)        # pitch center at arousal=0
        self.declare_parameter("base_dur_sec", 0.09)    # blip length at arousal=0
        self.declare_parameter("base_gap_sec", 0.04)    # silence between blips
        self.declare_parameter("base_gain", 0.6)        # loudness at dominance=0

        # --- PAD coupling ---
        self.declare_parameter("arousal_octaves", 0.5)    # arousal=+1 raises pitch this many octaves
        self.declare_parameter("arousal_speed", 0.4)      # arousal=+1 shortens blips by this fraction
        self.declare_parameter("pleasure_drift", 0.08)    # pleasure=+1 drifts pitch up this fraction over the line
        self.declare_parameter("dominance_gain", 0.25)    # dominance=+1 adds this to gain
        self.declare_parameter("dominance_tighten", 0.5)  # dominance=+1 shrinks gaps by this fraction

        # --- Melody ---
        # Semitone offsets a syllable's leading letter can land on. A
        # pentatonic-ish set so no word ever sounds sour. Same letter always
        # picks the same step - that's what keeps her voice recognizable.
        self.declare_parameter("pitch_steps", [0, 2, 4, 7, 9, 12])
        # Blips ending a sentence are held slightly longer.
        self.declare_parameter("terminal_stretch", 1.3)

        self.base_hz = self.get_parameter("base_hz").value
        self.base_dur = self.get_parameter("base_dur_sec").value
        self.base_gap = self.get_parameter("base_gap_sec").value
        self.base_gain = self.get_parameter("base_gain").value
        self.arousal_octaves = self.get_parameter("arousal_octaves").value
        self.arousal_speed = self.get_parameter("arousal_speed").value
        self.pleasure_drift = self.get_parameter("pleasure_drift").value
        self.dominance_gain = self.get_parameter("dominance_gain").value
        self.dominance_tighten = self.get_parameter("dominance_tighten").value
        self.pitch_steps = list(self.get_parameter("pitch_steps").value)
        self.terminal_stretch = self.get_parameter("terminal_stretch").value

        self.speech_pub = self.create_publisher(Speech, "/kerfur/speech", 10)
        self.create_subscription(Utterance, "/kerfur/utterance", self.on_utterance, 10)

        self.get_logger().info(
            f"Voice online: base={self.base_hz:.0f}Hz dur={self.base_dur:.3f}s "
            f"steps={self.pitch_steps}"
        )

    def on_utterance(self, msg: Utterance):
        text = msg.text.strip()
        if not text:
            return

        score = self.build_score(text, msg.pleasure, msg.arousal, msg.dominance)
        syls, pitch, durs, gaps, gain = score

        out = Speech()
        out.text = text
        out.syllables = syls
        out.pitch_hz = pitch
        out.duration_sec = durs
        out.gap_sec = gaps
        out.gain = gain
        out.timbre = self.pick_timbre(msg)
        self.speech_pub.publish(out)

        self.get_logger().info(
            f'Scored: "{text}" [{len(syls)} syl, {min(pitch):.0f}-{max(pitch):.0f}Hz, '
            f"gain={gain:.2f}, PAD=({msg.pleasure:+.2f},{msg.arousal:+.2f},{msg.dominance:+.2f})]"
        )

    def build_score(self, text, p, a, d):
        """Text + PAD -> parallel arrays. Pure function of its inputs."""
        syls = syllabify(text)
        n = len(syls)

        # Whole-utterance modulation
        pitch_center = self.base_hz * (2.0 ** (a * self.arousal_octaves))
        dur = self.base_dur * (1.0 - self.arousal_speed * a)
        gap = self.base_gap * (1.0 - self.dominance_tighten * d)
        gain = clamp(self.base_gain + self.dominance_gain * d, 0.1, 1.0)

        pitch, durs, gaps = [], [], []
        for i, syl in enumerate(syls):
            # Letter-keyed step: deterministic per syllable.
            lead = next((c for c in syl.lower() if c.isalpha()), "x")
            step = self.pitch_steps[ord(lead) % len(self.pitch_steps)]
            hz = pitch_center * (2.0 ** (step / 12.0))

            # Pleasure contour: drift across the line, up for pleasant.
            frac = i / max(1, n - 1)
            hz *= 1.0 + p * self.pleasure_drift * frac

            terminal = syl.rstrip().endswith((".", "!", "?"))
            pitch.append(float(hz))
            durs.append(float(dur * (self.terminal_stretch if terminal else 1.0)))
            gaps.append(0.0 if i == n - 1 else float(gap))

        return syls, pitch, durs, gaps, float(gain)

    def pick_timbre(self, msg: Utterance) -> str:
        """Timbre selection hook. Static for now; mode and sulk state plug in
        here once they exist. Names must match the TIMBRES table in speech.js."""
        return "default"


def main():
    rclpy.init()
    node = VoiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
