# Canonical PAD (Pleasure-Arousal-Dominance) affective state
# Each axis in [-1.0, +1.0]

float32 pleasure
float32 arousal
float32 dominance

# Discrete mode that selects which vertex set is active
# Values: "default", "sleepy", "playful", "alert", "bonding", "sick"
string mode

# Timestamp set by publisher
builtin_interfaces/Time stamp
