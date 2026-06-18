#!/bin/bash
# Kerfur HEAD startup - native ROS2 via RoboStack (no container).
# Opens each node group in its own terminal WINDOW (more robust on PiOS than
# tabs, which render badly / inconsistently across lxterminal versions).
#
# Mirrors start_dev.sh in spirit: a shared SRC string establishes the
# environment in every spawned shell, because a conda env is per-shell and does
# NOT carry into newly spawned terminals - each one must activate it itself.

WS=~/kerfur_ws
ENV_NAME=ros_env

# Absolute path to micromamba. A spawned `bash -c` window does NOT source
# ~/.bashrc, so `micromamba` is not on PATH there - reference it directly.
MAMBA=~/.local/bin/micromamba

# --- FastAPI hub (NON-ROS, NON-conda) ----------------------------------------
# The hub serves Kerferface and is a plain Python/uvicorn process. It must NOT
# run inside the ROS conda env. EDIT these to match your real hub location/cmd.
HUB_DIR=~/kerfur_head
HUB_CMD="python3 -m uvicorn hub:app --host 0.0.0.0 --port 8000"
HUB_URL="http://localhost:8000"

# Kill any lingering ROS nodes from a previous session
pkill -f "ros2 run" 2>/dev/null
pkill -f "ros2 launch" 2>/dev/null
sleep 1

# --- Environment bootstrap, run at the top of EVERY spawned shell ------------
# 1. Initialise the micromamba shell hook. A non-interactive `bash -c` subshell
#    does NOT source ~/.bashrc, so `micromamba activate` would otherwise fail
#    with "run micromamba shell init first". The hook line fixes that.
# 2. Activate the env, THEN source the workspace overlay (order matters: the
#    overlay was generated against the env's ROS/Python and needs it active).
SRC="eval \"\$($MAMBA shell hook --shell bash)\" && \
$MAMBA activate $ENV_NAME && \
source $WS/install/setup.bash && \
export ROS_DOMAIN_ID=42"

# --- Detect an available terminal emulator -----------------------------------
# Each emulator has a different "run a command in a new window" syntax, so we
# detect what's installed and define a launch() wrapper accordingly.
if command -v lxterminal >/dev/null 2>&1; then
  launch() { lxterminal --title="$1" -e bash -c "$2; exec bash" & }
elif command -v gnome-terminal >/dev/null 2>&1; then
  launch() { gnome-terminal --title="$1" -- bash -c "$2; exec bash" & }
elif command -v x-terminal-emulator >/dev/null 2>&1; then
  launch() { x-terminal-emulator -T "$1" -e bash -c "$2; exec bash" & }
elif command -v xterm >/dev/null 2>&1; then
  launch() { xterm -T "$1" -e bash -c "$2; exec bash" & }
else
  echo "No known terminal emulator found (lxterminal/gnome-terminal/xterm)."
  echo "Edit this script's launch() to match your terminal."
  exit 1
fi

# --- FastAPI hub (NON-ROS) ---------------------------------------------------
# Runs in a plain shell - NO conda env, NO ROS sourcing. Started first so its
# port is bound before expression_bridge tries to connect.
launch "KERFUR HUB" \
  "cd $HUB_DIR && echo '=== FASTAPI HUB (Kerferface) ===' && $HUB_CMD"
sleep 2   # give the hub a moment to bind its port before the bridge connects

# --- Head ROS stack: perception + expression bridge + touch bridge -----------
# All three come up from the head launch file.
launch "KERFUR HEAD" \
  "$SRC && echo '=== KERFUR HEAD STACK ===' && \
   ros2 launch kerfur_head head.launch.py"
sleep 2

# --- Browser window showing the face (NON-ROS) -------------------------------
# Plain shell. --app gives a chromeless window. The extra flags suppress
# Chromium's cloud phone-home noise (GCM registration errors etc.) that is
# harmless but spams the window. Flag order matters: all flags BEFORE the
# trailing `&` (the dev script's `&` was misplaced and dropped --user-data-dir).
launch "KERFUR FACE" \
  "echo '=== KERFUR FACE (browser) ===' && \
   chromium --app=$HUB_URL --user-data-dir=/tmp/kerfur-face \
     --disable-features=Translate,MediaRouter \
     --disable-sync --no-first-run --disable-background-networking \
     --password-store=basic 2>/dev/null"
sleep 1

# --- Free test console (sourced, ready to echo topics / fire events) ---------
launch "HEAD TEST" \
  "$SRC && echo '=== HEAD TEST CONSOLE ===' && \
   echo 'Watch detections:' && \
   echo '  ros2 topic echo /head/detection' && \
   echo '' && echo 'Watch nudges (touch reflex):' && \
   echo '  ros2 topic echo /kerfur/pad_nudge' && \
   echo ''"

echo "Kerfur head environment starting in separate terminal windows."
echo "If perception logs 'Hailo stack not importable', the Hailo bindings are"
echo "not yet reachable from the $ENV_NAME conda env (expected until bridged)."
