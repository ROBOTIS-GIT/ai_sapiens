#!/usr/bin/env bash
# Launch ROS2 processes in tmux

set -e

# Pull the --sim flag out of the arguments; the first remaining one names the
# tmux session.
SIM=false
ARGS=()
for arg in "$@"; do
  case "$arg" in
    "--sim") SIM=true ;;
    *) ARGS+=("$arg") ;;
  esac
done

SESSION_NAME="${ARGS[0]:-ai_sapiens}"

BRINGUP_LAUNCH="k1.launch.py"
if [ "$SIM" = true ]; then
  BRINGUP_LAUNCH="k1_mujoco.launch.py"
fi

hold_cmd() {
  local cmd="$1"
  printf 'bash -lc %q' "$cmd; status=\$?; echo; echo \"[tmux] command exited with status \$status. Press Ctrl-D or type exit to close this pane.\"; exec bash -i"
}

# Kill existing session if present
tmux has-session -t "$SESSION_NAME" 2>/dev/null && tmux kill-session -t "$SESSION_NAME"

# Create session with first command (top-left) — no delay
tmux new-session -d -s "$SESSION_NAME" -- "$(hold_cmd 'ros2 run rmw_zenoh_cpp rmw_zenohd')"

# Split right: top-right — sleep 3s then launch
tmux split-window -h -t "$SESSION_NAME" -- "$(hold_cmd "sleep 3; ros2 launch ai_sapiens_bringup ${BRINGUP_LAUNCH}")"

# Select top-left and split down: bottom-left — sleep 6s then launch
tmux select-pane -t "$SESSION_NAME":0.0
tmux split-window -v -t "$SESSION_NAME" -- "$(hold_cmd 'sleep 6; ros2 launch ai_sapiens_sim2real ai_sapiens_sim2real.launch.py')"

# Optional: balance pane sizes and attach
tmux select-layout -t "$SESSION_NAME" tiled
echo "Session '$SESSION_NAME' created. Attach with: tmux attach -t $SESSION_NAME"
tmux attach -t "$SESSION_NAME"
