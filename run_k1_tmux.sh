#!/usr/bin/env bash
# Launch ROS2 processes in tmux

set -e

SIM=false
RADIOMASTER_USB=false
RADIOMASTER_USB_DEVICE="/dev/input/js0"
RADIOMASTER_USB_DEVICE_SET=false
DUALSENSE=false
KEYBOARD=false
ARGS=()
while [ "$#" -gt 0 ]; do
  case "$1" in
    "--sim")
      SIM=true
      ;;
    "--radiomaster-usb")
      RADIOMASTER_USB=true
      ;;
    "--radiomaster-usb-device")
      if [ "$#" -lt 2 ]; then
        echo "Error: --radiomaster-usb-device requires a device path." >&2
        exit 2
      fi
      RADIOMASTER_USB_DEVICE="$2"
      RADIOMASTER_USB_DEVICE_SET=true
      shift
      ;;
    "--radiomaster-usb-device="*)
      RADIOMASTER_USB_DEVICE="${1#*=}"
      RADIOMASTER_USB_DEVICE_SET=true
      ;;
    "--dualsense")
      DUALSENSE=true
      ;;
    "--keyboard")
      KEYBOARD=true
      ;;
    "-h"|"--help")
      echo "Usage: $0 [session_name] [--sim] [--radiomaster-usb] [--radiomaster-usb-device=PATH] [--dualsense] [--keyboard]"
      echo
      echo "  --sim                         Launch the MuJoCo sim2sim bringup."
      echo "  --radiomaster-usb             Use RadioMaster USB RC input in MuJoCo."
      echo "  --radiomaster-usb-device=PATH Linux joystick device (default: /dev/input/js0)."
      echo "  --dualsense                   Use the DualSense teleop plugin and dashboard."
      echo "  --keyboard                    Use the keyboard teleop plugin and publisher."
      exit 0
      ;;
    "--"*)
      echo "Error: unknown option '$1'." >&2
      exit 2
      ;;
    *)
      ARGS+=("$1")
      ;;
  esac
  shift
done

if [ "${#ARGS[@]}" -gt 1 ]; then
  echo "Error: only one tmux session name may be specified." >&2
  exit 2
fi

if [ "$RADIOMASTER_USB" = true ] && [ "$SIM" != true ]; then
  echo "Error: --radiomaster-usb requires --sim." >&2
  exit 2
fi

if [ "$RADIOMASTER_USB_DEVICE_SET" = true ] && [ "$RADIOMASTER_USB" != true ]; then
  echo "Error: --radiomaster-usb-device requires --radiomaster-usb." >&2
  exit 2
fi

if [ "$RADIOMASTER_USB_DEVICE_SET" = true ] && [ -z "$RADIOMASTER_USB_DEVICE" ]; then
  echo "Error: --radiomaster-usb-device requires a non-empty device path." >&2
  exit 2
fi

if [ "$DUALSENSE" = true ] && [ "$KEYBOARD" = true ]; then
  echo "Error: --dualsense and --keyboard select the same teleop plugin slot; pick one." >&2
  exit 2
fi

SESSION_NAME="${ARGS[0]:-ai_sapiens}"

BRINGUP_LAUNCH="k1.launch.py"
if [ "$SIM" = true ]; then
  BRINGUP_LAUNCH="k1_mujoco.launch.py"
fi

BRINGUP_CMD="sleep 3; ros2 launch ai_sapiens_bringup ${BRINGUP_LAUNCH}"
if [ "$RADIOMASTER_USB" = true ]; then
  printf -v RADIOMASTER_USB_DEVICE_QUOTED '%q' "$RADIOMASTER_USB_DEVICE"
  BRINGUP_CMD+=" radiomaster_usb:=true"
  BRINGUP_CMD+=" radiomaster_usb_device:=${RADIOMASTER_USB_DEVICE_QUOTED}"
fi

SIM2REAL_COMMAND="sleep 6; ros2 launch ai_sapiens_sim2real ai_sapiens_sim2real.launch.py"
if [ "$DUALSENSE" = true ]; then
  DUALSENSE_CONFIG="$(ros2 pkg prefix --share ai_sapiens_sim2real)/config/teleop/dualsense.yaml"
  SIM2REAL_COMMAND+=" teleop_input_plugin:=ai_sapiens_sim2real/DualSenseTeleopInputPlugin"
  SIM2REAL_COMMAND+=" teleop_input_config_path:=${DUALSENSE_CONFIG}"
elif [ "$KEYBOARD" = true ]; then
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
    "$(hold_cmd "$BRINGUP_CMD")"
)"

# Select top-left and split down: bottom-left — sleep 6s then launch
tmux select-pane -t "$SESSION_NAME":0.0
tmux split-window -v -t "$SESSION_NAME" -- "$(hold_cmd "$SIM2REAL_COMMAND")"

if [ "$DUALSENSE" = true ]; then
  # Run joy_node and the DualSense dashboard only when explicitly requested.
  tmux split-window -v -t "$RIGHT_PANE" -- \
    "$(hold_cmd 'sleep 3; ros2 run ai_sapiens_sim2real run_dualsense_teleop.sh')"
fi

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
