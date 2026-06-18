# The single most-salient semantic detection from one camera.
# Produced by a perception node (per-camera, namespaced /head/ or /chest/).
# Consumed by the attention selector (behavior layer) to drive gaze.
# Singular by design: a creature attends to one thing at a time (CONVENTIONS §3.4).
# Absence of a salient target is published as label "" so subscribers idle / age-out.

string label            # "person" | "face" | "hand" | "" (= nothing salient)
float32 x               # horizontal position, [-1, +1], screen-relative (matches lookAt)
float32 y               # vertical position, [-1, +1], screen-relative
float32 size            # fraction of frame filled, [0, 1] - proximity proxy
float32 confidence      # detector confidence, [0, 1]
builtin_interfaces/Time stamp
