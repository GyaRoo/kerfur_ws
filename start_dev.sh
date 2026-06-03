#!/bin/bash
# Kerfur development environment startup
# Launches the FastAPI hub and all ROS2 nodes in separate Konsole tabs.

WS=~/kerfur_ws
SRC="source /opt/ros/jazzy/setup.bash && source $WS/install/setup.bash"

# --- FastAPI hub (NON-ROS) ---
# EDIT THIS LINE to match how you actually start your hub.
# Example placeholders - replace with your real command and path:
HUB_DIR=~/kerfur_head          # <-- directory containing your hub
HUB_CMD="python3 -m uvicorn hub:app --host 0.0.0.0 --port 8000 --reload"       # <-- command that starts the FastAPI hub

konsole --new-tab -p tabtitle="HUB" -e bash -c \
  "cd $HUB_DIR && echo '=== FASTAPI HUB ===' && $HUB_CMD; exec bash" &
sleep 2   # give the hub a moment to bind its port before the bridge connects

# --- Emotion engine (continuous PAD source) ---
konsole --new-tab -p tabtitle="ENGINE" -e bash -c \
  "$SRC && echo '=== EMOTION ENGINE ===' && ros2 run kerfur_behavior emotion_engine; exec bash" &
sleep 1

# --- Behavior node (PAD -> Expression) ---
konsole --new-tab -p tabtitle="BEHAVIOR" -e bash -c \
  "$SRC && echo '=== BEHAVIOR ===' && ros2 run kerfur_behavior pad_to_face; exec bash" &
sleep 1

# --- Bridge (Expression -> hub WebSocket) ---
konsole --new-tab -p tabtitle="BRIDGE" -e bash -c \
  "$SRC && echo '=== BRIDGE ===' && ros2 run kerfur_bridge expression_bridge; exec bash" &
sleep 1

# --- Subconscious (Situation -> PAD nudge) ---
konsole --new-tab -p tabtitle="SUBCON" -e bash -c \
  "$SRC && echo '=== SUBCONSCIOUS ===' && ros2 run kerfur_cognition subconscious; exec bash" &
sleep 1

# --- Free test console (sourced, ready to fire situations/nudges) ---
konsole --new-tab -p tabtitle="TEST" -e bash -c \
  "$SRC && echo '=== TEST CONSOLE ===' && \
   echo 'Fire a situation:' && \
   echo '  ros2 topic pub --once /kerfur/situation kerfur_msgs/msg/Situation \"{description: \\\"someone gently pets the robot\\\"}\"' && \
   echo '' && echo 'Fire a raw nudge:' && \
   echo '  ros2 topic pub --once /kerfur/pad_nudge kerfur_msgs/msg/PADNudge \"{d_pleasure: 0.5, d_arousal: 0.3, d_dominance: 0.1, reason: test}\"' && \
   exec bash" &
# --- Open browser ---
sleep 2
chromium --app=http://localhost:8000 & --user-data-dir=/tmp/kerfur-face

echo "Kerfur dev environment starting in separate Konsole tabs."
echo "If the BRIDGE tab shows 'Hub connection failed', the hub line in this script needs fixing."
