# Where Kerfur's eyes should point. Produced by the attention selector,
# forwarded by the face bridge to the browser's lookAt(x, y).
# Screen-relative, normalized: 0,0 = center, matches the face's lookAt convention.

float32 x               # horizontal gaze, [-1, +1] (+ = viewer's right in screen space)
float32 y               # vertical gaze, [-1, +1] (+ = down)

builtin_interfaces/Time stamp
