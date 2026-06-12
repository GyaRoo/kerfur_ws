"""Voice subsystem: speaks utterances in a synthesizer-flavored, mood-colored voice.

Subscribes to /kerfur/utterance, synthesizes via Piper, applies mood-coloring
(speed/pitch from PAD), plays to a configured audio sink. Identical on M80q and
head - the audio sink parameter is the only deployment difference. The DSP
'synthesizer character' chain is a tuning hook to fill in over time.
"""

import os
import subprocess
import tempfile
import threading
import queue
import wave

import rclpy
from rclpy.node import Node

from kerfur_msgs.msg import Utterance


class VoiceNode(Node):
    def __init__(self):
        super().__init__("voice")

        # --- Parameters (go in the global config; differ per deployment) ---
        # Piper binary + voice model
        self.declare_parameter("piper_bin", "piper")
        self.declare_parameter("piper_model", "/home/roo/piper/voice.onnx")
        # Audio playback command. {wav} is substituted with the file path.
        # M80q: default sink. Head: point at the USB DAC / BT sink via aplay -D.
        self.declare_parameter("play_cmd", "aplay {wav}")
        # Mood-coloring ranges (tunable)
        self.declare_parameter("length_scale_base", 1.0)   # Piper pacing; >1 slower
        self.declare_parameter("length_scale_arousal", -0.3)  # arousal speeds speech
        self.declare_parameter("pitch_semitones_arousal", 2.0)  # arousal raises pitch

        self.piper_bin = self.get_parameter("piper_bin").value
        self.piper_model = self.get_parameter("piper_model").value
        self.play_cmd = self.get_parameter("play_cmd").value
        self.ls_base = self.get_parameter("length_scale_base").value
        self.ls_arousal = self.get_parameter("length_scale_arousal").value
        self.pitch_arousal = self.get_parameter("pitch_semitones_arousal").value

        # Speech runs in a worker thread so synthesis/playback never blocks ROS spin.
        # A queue serializes utterances - Kerfur says one thing at a time.
        self.speech_queue = queue.Queue()
        self.worker = threading.Thread(target=self._speech_worker, daemon=True)
        self.worker.start()

        self.create_subscription(Utterance, "/kerfur/utterance", self.on_utterance, 10)

        self.get_logger().info(
            f"Voice online: piper={self.piper_bin}, model={self.piper_model}, "
            f"play='{self.play_cmd}'"
        )

    def on_utterance(self, msg: Utterance):
        if not msg.text.strip():
            return
        # Hand off to the worker; don't synthesize on the ROS callback thread.
        self.speech_queue.put(msg)
        self.get_logger().info(f'Queued utterance: "{msg.text}"')

    def _speech_worker(self):
        while True:
            msg = self.speech_queue.get()
            try:
                self._speak(msg)
            except Exception as e:
                self.get_logger().warn(f"Voice synthesis/playback failed: {e}")
            finally:
                self.speech_queue.task_done()

    def _speak(self, msg: Utterance):
        # --- Mood -> Piper pacing ---
        # arousal in [-1,1]; higher arousal -> faster speech (lower length_scale)
        length_scale = self.ls_base + self.ls_arousal * msg.arousal
        length_scale = max(0.5, min(2.0, length_scale))  # clamp to sane range

        # --- Synthesize with Piper to a temp WAV ---
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tf:
            wav_path = tf.name
        try:
            # Piper reads text on stdin, writes WAV to --output_file
            piper_cmd = [
                self.piper_bin,
                "--model", self.piper_model,
                "--length_scale", str(length_scale),
                "--output_file", wav_path,
            ]
            subprocess.run(
                piper_cmd,
                input=msg.text.encode("utf-8"),
                check=True,
                capture_output=True,
            )

            # --- Mood -> pitch + (future) DSP character chain ---
            # arousal raises pitch; this is the hook where the synthesizer
            # character (formant/ring-mod/vocoder) gets tuned in later.
            pitch_semitones = self.pitch_arousal * msg.arousal
            processed = self._apply_character(wav_path, pitch_semitones, msg)

            # --- Play to the configured sink ---
            play = self.play_cmd.format(wav=processed)
            subprocess.run(play, shell=True, check=True, capture_output=True)

            self.get_logger().info(
                f'Spoke: "{msg.text}" '
                f"[ls={length_scale:.2f}, pitch={pitch_semitones:+.1f}st, "
                f"PAD=({msg.pleasure:+.2f},{msg.arousal:+.2f},{msg.dominance:+.2f})]"
            )
        finally:
            for p in {wav_path, locals().get("processed", wav_path)}:
                try:
                    if p and os.path.exists(p):
                        os.remove(p)
                except OSError:
                    pass

    def _apply_character(self, wav_path, pitch_semitones, msg):
        """Apply pitch shift + synthesizer character.

        FIRST VERSION: pitch shift only, via sox if available; otherwise pass through.
        This is the tuning hook for the 'cat with a synthesizer voice' character -
        formant shift, ring mod, vocoder, etc. get layered here over time.
        Returns the path to the processed WAV (or the original if no processing).
        """
        # If no pitch change requested and no character chain yet, pass through.
        if abs(pitch_semitones) < 0.01:
            return wav_path

        out_path = wav_path.replace(".wav", "_proc.wav")
        try:
            # sox pitch shift in cents (semitones * 100). Requires sox installed.
            subprocess.run(
                ["sox", wav_path, out_path, "pitch", str(int(pitch_semitones * 100))],
                check=True,
                capture_output=True,
            )
            return out_path
        except (subprocess.CalledProcessError, FileNotFoundError):
            # sox missing or failed - pass through unprocessed rather than fail.
            self.get_logger().warn("sox unavailable; skipping pitch/character processing")
            return wav_path


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
