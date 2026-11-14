"""Voice subsystem: turns utterances into blabber scores.

Subscribes to /kerfur/utterance, builds a per-letter score (pitch,
duration, gap) colored by the PAD snapshot the utterance carries, and
publishes it as a Speech message. Nothing is synthesized here - the face
browser plays the score and reveals the text in step.

Python decides what she sounds like; the browser only plays it. Every
coefficient is a parameter, read per utterance, so the voice can be tuned
live with `ros2 param set /voice ...`.
"""

import rclpy
from rclpy.node import Node

from kerfur_msgs.msg import Utterance, Speech


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def is_sounded(ch: str) -> bool:
    """Only A-Z has a recording in the letter library."""
    return ch.isascii() and ch.isalpha()


def letter_units(text: str, shorten: bool):
    """Split text into display chunks, one blip each.

    Returns a list of (chunk, word_end, terminal). Joined, the chunks
    reproduce the text, so the overlay can reveal them in order. The first
    A-Z letter in a chunk is the one the browser sounds; punctuation and
    digits ride along silently in a neighbouring chunk.

    With shorten=True only the first and last letter of each word are
    sounded (two blips per word), but the chunks still carry the whole
    word, so the overlay text is unaffected.
    """
    units = []
    pending = ""   # letterless tokens seen before the first sounded chunk
    for word in text.split(" "):
        if not word:
            continue
        alpha = [i for i, c in enumerate(word) if is_sounded(c)]

        if not alpha:
            # No letters ("42", "-"): attach to the previous chunk, or
            # hold for the next one if nothing has been sounded yet.
            if units:
                chunk, _, term = units[-1]
                units[-1] = (chunk + word + " ", True, term)
            else:
                pending += word + " "
            continue

        keep = [alpha[0], alpha[-1]] if (shorten and len(alpha) > 2) else alpha
        terminal = word.rstrip("\"')").endswith((".", "!", "?"))

        for k, start in enumerate(keep):
            last = (k == len(keep) - 1)
            lo = 0 if k == 0 else start
            hi = len(word) if last else keep[k + 1]
            chunk = word[lo:hi]
            if k == 0:
                chunk = pending + chunk
                pending = ""
            if last:
                chunk += " "
            units.append((chunk, last, last and terminal))

    if not units:
        # Nothing soundable at all: one silent beat that shows the text.
        return [(text, True, False)]

    chunk, word_end, term = units[-1]
    units[-1] = (chunk.rstrip(), word_end, term)
    return units


class VoiceNode(Node):
    def __init__(self):
        super().__init__("voice")

        # --- Patch ---
        # Must match a name in the TIMBRES table in speech.js.
        self.declare_parameter("timbre", "animalese")
        # Sound only the first and last letter of each word.
        self.declare_parameter("shorten_words", False)

        # --- Base voice ---
        # With the sampled patch, 440 plays the library at native speed.
        self.declare_parameter("base_hz", 440.0)
        self.declare_parameter("base_dur_sec", 0.075)   # per letter at arousal=0
        self.declare_parameter("base_gap_sec", 0.0)     # between letters in a word
        self.declare_parameter("word_gap_sec", 0.075)   # extra silence after a word
        self.declare_parameter("base_gain", 0.6)

        # --- PAD coupling ---
        self.declare_parameter("arousal_octaves", 0.5)
        self.declare_parameter("arousal_speed", 0.4)
        self.declare_parameter("pleasure_drift", 0.08)
        self.declare_parameter("dominance_gain", 0.25)
        self.declare_parameter("dominance_tighten", 0.5)

        # --- Melody ---
        # Semitone offsets keyed by letter. [0] = no per-letter hopping,
        # which is what the sampled patch wants.
        self.declare_parameter("pitch_steps", [0])
        self.declare_parameter("terminal_stretch", 1.3)

        self.speech_pub = self.create_publisher(Speech, "/kerfur/speech", 10)
        self.create_subscription(Utterance, "/kerfur/utterance", self.on_utterance, 10)

        self.get_logger().info(
            f"Voice online: timbre={self._p('timbre')} "
            f"base={self._p('base_hz'):.0f}Hz dur={self._p('base_dur_sec'):.3f}s"
        )

    def _p(self, name):
        """Read a parameter now, so `ros2 param set` takes effect on the
        next utterance."""
        return self.get_parameter(name).value

    def on_utterance(self, msg: Utterance):
        text = msg.text.strip()
        if not text:
            return

        chunks, pitch, durs, gaps, gain = self.build_score(
            text, msg.pleasure, msg.arousal, msg.dominance
        )

        out = Speech()
        out.text = text
        out.syllables = chunks
        out.pitch_hz = pitch
        out.duration_sec = durs
        out.gap_sec = gaps
        out.gain = gain
        out.timbre = self._p("timbre")
        self.speech_pub.publish(out)

        self.get_logger().info(
            f'Scored: "{text}" [{len(chunks)} blips, {min(pitch):.0f}-{max(pitch):.0f}Hz, '
            f"{sum(durs) + sum(gaps):.2f}s, gain={gain:.2f}, "
            f"PAD=({msg.pleasure:+.2f},{msg.arousal:+.2f},{msg.dominance:+.2f})]"
        )

    def build_score(self, text, p, a, d):
        """Text + PAD -> parallel arrays."""
        units = letter_units(text, bool(self._p("shorten_words")))
        n = len(units)

        # Whole-utterance modulation
        pitch_center = self._p("base_hz") * (2.0 ** (a * self._p("arousal_octaves")))
        dur = self._p("base_dur_sec") * (1.0 - self._p("arousal_speed") * a)
        tighten = 1.0 - self._p("dominance_tighten") * d
        gap = self._p("base_gap_sec") * tighten
        word_gap = self._p("word_gap_sec") * tighten
        gain = clamp(self._p("base_gain") + self._p("dominance_gain") * d, 0.1, 1.0)

        steps = list(self._p("pitch_steps")) or [0]
        drift = self._p("pleasure_drift")
        stretch = self._p("terminal_stretch")

        chunks, pitch, durs, gaps = [], [], [], []
        for i, (chunk, word_end, terminal) in enumerate(units):
            lead = next((c for c in chunk.lower() if is_sounded(c)), "x")
            step = steps[ord(lead) % len(steps)]
            hz = pitch_center * (2.0 ** (step / 12.0))
            # Pleasure contour: drift across the line, up for pleasant.
            hz *= 1.0 + p * drift * (i / max(1, n - 1))

            chunks.append(chunk)
            pitch.append(float(hz))
            durs.append(float(dur * (stretch if terminal else 1.0)))
            g = gap + (word_gap if word_end else 0.0)
            gaps.append(0.0 if i == n - 1 else float(max(0.0, g)))

        return chunks, pitch, durs, gaps, float(gain)


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
