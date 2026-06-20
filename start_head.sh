#!/bin/bash
# Kerfur HEAD startup - native ROS2 via RoboStack (no container).
# Opens each node group in its own terminal WINDOW (more robust on PiOS than
# tabs, which render badly / inconsistently across lxterminal versions).
#
# Mirrors start_dev.sh in spirit: a shared SRC string establishes the
# environment in every spawned shell, because a conda env is per-shell and does
# NOT carry into newly spawned terminals - each one must activate it itself.

WS=$HOME/kerfur_ws
ENV_NAME=ros_env

# Absolute paths, $HOME not ~ (tilde does not expand reliably inside the quoted
# strings we pass through `bash -c`, which was causing micromamba to fall back
# to its WRONG compiled-in default root prefix and fault with "Shell not
# initialized"). Use $HOME everywhere and set the real root prefix explicitly.
MAMBA=$HOME/.local/bin/micromamba
export MAMBA_ROOT_PREFIX=$HOME/micromamba

# --- FastAPI hub (NON-ROS, NON-conda) ----------------------------------------
# The hub serves Kerferface and is a plain Python/uvicorn process. It must NOT
# run inside the ROS conda env. EDIT these to match your real hub location/cmd.
HUB_DIR=$HOME/kerfur_head
HUB_CMD="python3 -m uvicorn hub:app --host 0.0.0.0 --port 8000"
HUB_URL="http://localhost:8000"

# Kill any lingering ROS nodes from a previous session
pkill -f "ros2 run" 2>/dev/null
pkill -f "ros2 launch" 2>/dev/null
sleep 1

# --- Environment runner ------------------------------------------------------
# We use `micromamba run` rather than `activate`. `run` executes a command
# INSIDE the env without initialising a shell - no hook, no activate, no
# "Shell not initialized" class of error. Each ROS launch below inlines its
# own `micromamba run -n ros_env bash -c '...'` so nothing depends on shell
# state or exported functions surviving the terminal-emulator boundary.

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
# Self-contained: micromamba run executes the launch inside the env without
# initialising a shell. $HOME (not ~) so the path expands in every context.
launch "KERFUR HEAD" \
  "echo '=== KERFUR HEAD STACK ===' && \
   MAMBA_ROOT_PREFIX=$HOME/micromamba $MAMBA run -n $ENV_NAME \
     bash -c 'source $WS/install/setup.bash && export ROS_DOMAIN_ID=42 && \
              ros2 launch kerfur_head head.launch.py'"
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
     --password-store=basic 2>/dev/null\
     --start-fullscreen"
sleep 1

# --- Free test console (in-env shell, ready to echo topics / fire events) ----
# Drops into an interactive bash INSIDE the env via micromamba run, with the
# overlay already sourced, so you can type ros2 commands directly.
launch "HEAD TEST" \
  "echo '=== HEAD TEST CONSOLE ===' && \
   echo 'Watch detections:   ros2 topic echo /head/detection' && \
   echo 'Watch nudges:        ros2 topic echo /kerfur/pad_nudge' && \
   echo '' && \
   MAMBA_ROOT_PREFIX=$HOME/micromamba $MAMBA run -n $ENV_NAME \
     bash --rcfile <(echo 'source $WS/install/setup.bash; export ROS_DOMAIN_ID=42')"

echo "Kerfur head environment starting in separate terminal windows."
echo "If perception logs 'Hailo stack not importable', the Hailo bindings are"
echo "not yet reachable from the $ENV_NAME conda env (expected until bridged)."
