#!/usr/bin/env bash
# Launch ROS2 processes in tmux

set -e

# Pull the --keyboard flag out of the arguments; the first remaining argument
# names the tmux session.
KEYBOARD=false
ARGS=()
for arg in "$@"; do
  case "$arg" in
    "--keyboard") KEYBOARD=true ;;
    *) ARGS+=("$arg") ;;
  esac
done

SESSION_NAME="${ARGS[0]:-ai_sapiens}"
SIM2REAL_COMMAND="sleep 6; ros2 launch ai_sapiens_sim2real ai_sapiens_sim2real.launch.py"
if [ "$KEYBOARD" = true ]; then
  KEYBOARD_CONFIG="$(ros2 pkg prefix --share ai_sapiens_sim2real)/config/teleop/keyboard.yaml"
  SIM2REAL_COMMAND+=" teleop_input_plugin:=ai_sapiens_sim2real/KeyboardTeleopInputPlugin"
  SIM2REAL_COMMAND+=" teleop_input_config_path:=${KEYBOARD_CONFIG}"
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
RIGHT_PANE="$(
  tmux split-window -h -t "$SESSION_NAME" -P -F '#{pane_id}' -- \
    "$(hold_cmd 'sleep 3; ros2 launch ai_sapiens_bringup k1.launch.py')"
)"

# Select top-left and split down: bottom-left — sleep 6s then launch
tmux select-pane -t "$SESSION_NAME":0.0
tmux split-window -v -t "$SESSION_NAME" -- "$(hold_cmd "$SIM2REAL_COMMAND")"

if [ "$KEYBOARD" = true ]; then
  # Bottom-right stays attached to a TTY so raw key presses and arrow-key
  # escape sequences reach the interactive keyboard publisher.
  tmux split-window -v -t "$RIGHT_PANE" -- \
    "$(hold_cmd 'sleep 3; ros2 run ai_sapiens_sim2real keyboard_teleop_node')"
fi

# Optional: balance pane sizes and attach
tmux select-layout -t "$SESSION_NAME" tiled
echo "Session '$SESSION_NAME' created. Attach with: tmux attach -t $SESSION_NAME"
tmux attach -t "$SESSION_NAME"
