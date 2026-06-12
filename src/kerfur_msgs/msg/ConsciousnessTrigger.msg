# A trigger that wakes the consciousness layer.
# trigger_type: "address" (spoken to / queried), "event" (HA/sensor escalation),
#               "novelty" (subconscious flagged something beyond it)
string trigger_type

# The content - what was said, what happened, etc.
string content

# Optional source tag for logging/context (e.g. "user", "home_assistant", "subconscious")
string source
