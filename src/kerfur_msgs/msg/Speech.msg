string text                 # full line, for the overlay
string[] syllables          # one entry per blip, for reveal pacing
float32[] pitch_hz          # per blip
float32[] duration_sec      # per blip, sound-on time
float32[] gap_sec           # per blip, silence after
float32 gain                # 0..1, whole utterance
string timbre               # "default" | "sulk" | "sleepy" ... picks the browser patch
