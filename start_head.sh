#!/bin/bash
# Kerfur HEAD startup - native ROS2 via RoboStack (no container).
# Opens each node group in its own terminal WINDOW (more robust on PiOS than
# tabs, which render badly / inconsistently across lxterminal versions).
#
# Mirrors start_dev.sh in spirit: a shared ROS_ENV string establishes the
# environment in every spawned shell, because a conda env is per-shell and does
# NOT carry into newly spawned terminals - each one must activate it itself.
#
# ---------------------------------------------------------------------------
# RULES (learned the hard way - keep these or the ghosts come back):
#   1. Absolute paths via $HOME, never ~. Tilde does not expand reliably inside
#      quoted strings passed through nested `bash -c`.
#   2. Use $MAMBA (absolute path), never bare `micromamba`. PATH in a spawned
#      terminal is not trustworthy.
#   3. ROS_DOMAIN_ID and FASTRTPS_DEFAULT_PROFILES_FILE go ON the `micromamba
#      run` line as prefix assignments, never exported inside the inner bash -c.
#      Exports inside the nested quotes got dropped before.
#   4. EVERY shell that runs `ros2` (stack AND test console) must use the same
#      $ROS_ENV. The first `ros2` CLI command on this machine spawns the daemon,
#      which inherits that shell's environment. A test console without the
#      peers file spawns a daemon that cannot see the dev machine.
#   5. The daemon survives script restarts. It is killed explicitly below.
# ---------------------------------------------------------------------------

WS=$HOME/kerfur_ws
# rc file for the test console. A real file, not <(...) process substitution -
# micromamba run does not pass the fd through, and bash silently ignores a
# missing rcfile, giving a shell with NO overlay and no error.
RCFILE=$HOME/.kerfur_head_rc
cat > "$RCFILE" <<EOF
[ -f "\$HOME/.bashrc" ] && source "\$HOME/.bashrc"
source $WS/install/setup.bash
PS1="(kerfur) \$PS1"
EOF




ENV_NAME=ros_env
MAMBA=$HOME/.local/bin/micromamba
export MAMBA_ROOT_PREFIX=$HOME/micromamba

# DDS config - static unicast peers because the WiFi AP drops multicast.
# The XML must contain the dev machine's IP (this Fast DDS build rejects
# hostnames) and the maxInitialPeersRange transport block.
PEERS_FILE=$HOME/fastdds_peers.xml
DOMAIN_ID=42

# Single source of truth for "run this inside the ROS env with the right DDS
# settings". Expanded once here, embedded literally into every launch string.
ROS_ENV="ROS_DOMAIN_ID=$DOMAIN_ID FASTRTPS_DEFAULT_PROFILES_FILE=$PEERS_FILE $MAMBA run -n $ENV_NAME"

# --- FastAPI hub (NON-ROS, NON-conda) ----------------------------------------
# The hub serves Kerferface and is a plain Python/uvicorn process. It must NOT
# run inside the ROS conda env.
HUB_DIR=$HOME/kerfur_head
HUB_CMD="python3 -m uvicorn hub:app --host 0.0.0.0 --port 8000"
HUB_URL="http://localhost:8000"

# --- Hailo detector (NON-ROS, runs in the HAILO venv, NOT conda) --------------
# The detector does YOLO on the Hailo and writes detections to a local socket
# that detection_bridge (in ros_env) reads. It lives in a DIFFERENT environment
# than the ROS nodes - the hailo-apps venv, not the conda env.
HAILO_DIR=$HOME/hailo-apps
HAILO_ENV_SETUP=setup_env.sh                  # sourced FROM that dir (relative)
DETECTOR_SCRIPT=$HOME/kerfur_detector.py
DETECTOR_INPUT="usb"                          # --input value (usb auto-detects)

# --- Sanity checks before spawning anything ----------------------------------
# Fail loud here, in THIS terminal, rather than in five windows at once.
fail() { echo "ERROR: $*" >&2; exit 1; }
[ -x "$MAMBA" ]                      || fail "micromamba not found at $MAMBA"
[ -f "$PEERS_FILE" ]                 || fail "peers file missing: $PEERS_FILE"
[ -f "$WS/install/setup.bash" ]      || fail "workspace not built: $WS/install/setup.bash missing (run colcon build)"

# --- Kill leftovers from a previous session ----------------------------------
pkill -f "ros2 run" 2>/dev/null
pkill -f "ros2 launch" 2>/dev/null
pkill -f "kerfur_detector.py" 2>/dev/null
# The daemon is per-domain and long-lived. Stop it properly (needs the domain
# ID to find the right one), then pkill as belt-and-braces.
$ROS_ENV ros2 daemon stop >/dev/null 2>&1
pkill -f _ros2_daemon 2>/dev/null
sleep 1

# --- Detect an available terminal emulator -----------------------------------
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
# Started first so its port is bound before expression_bridge tries to connect.
launch "KERFUR HUB" \
  "cd $HUB_DIR && echo '=== FASTAPI HUB (Kerferface) ===' && $HUB_CMD"
sleep 2

# --- Hailo detector (HAILO venv) ---------------------------------------------
launch "KERFUR DETECTOR" \
  "echo '=== HAILO DETECTOR (vision) ===' && \
   cd $HAILO_DIR && \
   source $HAILO_ENV_SETUP && \
   python3 $DETECTOR_SCRIPT --input $DETECTOR_INPUT"
sleep 3   # the Hailo pipeline takes a moment to spin up its GStreamer graph

# --- Head ROS stack ----------------------------------------------------------
# Self-reporting: prints the environment it actually has BEFORE launching.
# Chained with && so a broken env stops here with the error on screen instead
# of scrolling away under node output. \$ escapes survive to the inner bash so
# they expand in the process that actually launches the nodes.
launch "KERFUR HEAD" \
  "echo '=== KERFUR HEAD STACK ===' && \
   $ROS_ENV bash -c 'source $WS/install/setup.bash && \
     echo \"PYTHON: \$(which python3)\" && \
     echo \"PKG:    \$(ros2 pkg prefix kerfur_msgs)\" && \
     echo \"PEERS:  \$FASTRTPS_DEFAULT_PROFILES_FILE\" && \
     echo \"DOMAIN: \$ROS_DOMAIN_ID\" && \
     echo \"RMW:    \$(ros2 doctor --report 2>/dev/null | grep -i \"middleware name\" | head -1)\" && \
     echo '----------------------------------------' && \
     ros2 launch kerfur_head head.launch.py'"
sleep 2

# --- Browser window showing the face (NON-ROS) -------------------------------
launch "KERFUR FACE" \
  "echo '=== KERFUR FACE (browser) ===' && \
   chromium --app=$HUB_URL --user-data-dir=/tmp/kerfur-face \
     --disable-features=Translate,MediaRouter \
     --disable-sync --no-first-run --disable-background-networking \
     --password-store=basic --start-fullscreen 2>/dev/null"
sleep 1

# --- Free test console (in-env shell) ----------------------------------------
# SAME $ROS_ENV as the stack. Anything typed here, and any daemon it spawns,
# sees exactly what the nodes see.
launch "HEAD TEST" \
  "echo '=== HEAD TEST CONSOLE (domain $DOMAIN_ID, peers file loaded) ===' && \
   echo 'Watch detections:   ros2 topic echo /head/detection' && \
   echo 'Watch nudges:       ros2 topic echo /kerfur/pad_nudge' && \
   echo 'Graph:              ros2 node list' && \
   echo '' && \
   $ROS_ENV bash --rcfile $RCFILE"

echo "Kerfur head starting in separate terminal windows:"
echo "  HUB -> DETECTOR -> HEAD STACK -> FACE -> TEST"
echo "Read the PYTHON/PKG/PEERS/DOMAIN lines in the HEAD window first."
echo "If PKG is empty the launch is blocked - fix the env before anything else."
