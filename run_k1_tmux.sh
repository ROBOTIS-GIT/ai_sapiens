#!/usr/bin/env bash
# Launch the K1 ROS 2 stack in tmux.

set -e

# Runtime choices
SIM=false
TELEOP="default"
DEVICE="/dev/input/js0"
DEVICE_SET=false
SESSION_ARGS=()

die() {
  echo "Error: $*" >&2
  exit 2
}

set_teleop() {
  if [ "$TELEOP" != "default" ] && [ "$TELEOP" != "$1" ]; then
    die "teleop options are mutually exclusive."
  fi
  TELEOP="$1"
}

usage() {
  echo "Usage: $0 [session_name] [options]"
  echo "  --sim                  Use MuJoCo."
  echo "  --radiomaster-usb      Use RadioMaster USB (MuJoCo only)."
  echo "  --dualsense            Use DualSense."
  echo "  --keyboard             Use keyboard teleop."
  echo "  --device=/dev/input/jsN  Select the RadioMaster or DualSense device."
}

# Parse order-independent long options.
for arg in "$@"; do
  case "$arg" in
    --sim) SIM=true ;;
    --radiomaster-usb|--dualsense|--keyboard) set_teleop "${arg#--}" ;;
    --device=*) DEVICE="${arg#*=}"; DEVICE_SET=true ;;
    -h|--help) usage; exit 0 ;;
    --*) die "unknown option '$arg'." ;;
    *) SESSION_ARGS+=("$arg") ;;
  esac
done

if [ "${#SESSION_ARGS[@]}" -gt 1 ]; then
  die "only one session name may be specified."
fi
SESSION_NAME="${SESSION_ARGS[0]:-ai_sapiens}"

# Reject option combinations that cannot be launched together.
if [ "$TELEOP" = "radiomaster-usb" ] && [ "$SIM" != true ]; then
  die "--radiomaster-usb requires --sim."
fi

if [ "$DEVICE_SET" = true ]; then
  if [ "$TELEOP" != "radiomaster-usb" ] && [ "$TELEOP" != "dualsense" ]; then
    die "--device requires --radiomaster-usb or --dualsense."
  fi
fi
if [[ ! "$DEVICE" =~ ^/dev/input/js[0-9]+$ ]]; then
  die "--device must have the form /dev/input/jsN."
fi

# Build the commands that each pane will run.
BRINGUP_LAUNCH="k1.launch.py"
if [ "$SIM" = true ]; then
  BRINGUP_LAUNCH="k1_mujoco.launch.py"
fi

BRINGUP_CMD="sleep 3; ros2 launch ai_sapiens_bringup $BRINGUP_LAUNCH"
SIM2REAL_CMD="sleep 6; ros2 launch ai_sapiens_sim2real ai_sapiens_sim2real.launch.py"
TELEOP_CMD=""

use_plugin() {
  local plugin="$1"
  local config="$2"
  local share
  share="$(ros2 pkg prefix --share ai_sapiens_sim2real)"
  SIM2REAL_CMD+=" teleop_input_plugin:=ai_sapiens_sim2real/$plugin"
  SIM2REAL_CMD+=" teleop_input_config_path:=$share/config/teleop/$config.yaml"
}

printf -v DEVICE_QUOTED '%q' "$DEVICE"
case "$TELEOP" in
  radiomaster-usb)
    BRINGUP_CMD+=" radiomaster_usb:=true radiomaster_usb_device:=$DEVICE_QUOTED"
    ;;
  dualsense)
    use_plugin "DualSenseTeleopInputPlugin" "dualsense"
    TELEOP_CMD="sleep 3; ros2 run ai_sapiens_sim2real run_dualsense_teleop.sh --device=$DEVICE_QUOTED"
    ;;
  keyboard)
    use_plugin "KeyboardTeleopInputPlugin" "keyboard"
    TELEOP_CMD="sleep 3; ros2 run ai_sapiens_sim2real keyboard_teleop_node"
    ;;
esac

# Keep a pane open when its command exits so its logs remain visible.
hold_cmd() {
  local cmd="$1"
  printf 'bash -lc %q' "$cmd; status=\$?; echo; echo \"[tmux] command exited with status \$status. Press Ctrl-D or type exit to close this pane.\"; exec bash -i"
}

add_pane() {
  tmux split-window -t "$SESSION_NAME" -- "$(hold_cmd "$1")"
}

# Start Zenoh first, then add the K1 processes as tiled panes.
if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
  tmux kill-session -t "$SESSION_NAME"
fi
tmux new-session -d -s "$SESSION_NAME" -- \
  "$(hold_cmd 'ros2 run rmw_zenoh_cpp rmw_zenohd')"

add_pane "$BRINGUP_CMD"
add_pane "$SIM2REAL_CMD"
if [ -n "$TELEOP_CMD" ]; then
  add_pane "$TELEOP_CMD"
fi

# Add a custom pane here, for example:
# add_pane "ros2 run my_package my_node"

tmux select-layout -t "$SESSION_NAME" tiled
echo "Session '$SESSION_NAME' created. Attach with: tmux attach -t $SESSION_NAME"
tmux attach -t "$SESSION_NAME"
