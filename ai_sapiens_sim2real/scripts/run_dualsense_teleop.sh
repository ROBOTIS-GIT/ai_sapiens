#!/usr/bin/env bash

set -e

JOY_DEVICE="/dev/input/js0"
JOY_PID=""
UI_ARGS=()

while [ "$#" -gt 0 ]; do
  case "$1" in
    --device)
      if [ "$#" -lt 2 ]; then
        echo "Error: --device requires a device path." >&2
        exit 2
      fi
      JOY_DEVICE="$2"
      shift
      ;;
    --device=*)
      JOY_DEVICE="${1#*=}"
      ;;
    *)
      UI_ARGS+=("$1")
      ;;
  esac
  shift
done

if [[ ! "$JOY_DEVICE" =~ ^/dev/input/js([0-9]+)$ ]]; then
  echo "Error: --device must have the form /dev/input/jsN." >&2
  exit 2
fi
JOY_DEVICE_ID="${BASH_REMATCH[1]}"

cleanup() {
  if [ -n "$JOY_PID" ] && kill -0 "$JOY_PID" 2>/dev/null; then
    kill "$JOY_PID"
    wait "$JOY_PID" 2>/dev/null || true
  fi
}

trap cleanup EXIT INT TERM

ros2 run joy joy_node --ros-args \
  -p device_id:="$JOY_DEVICE_ID" \
  -p deadzone:=0.0 \
  --log-level warn &
JOY_PID=$!

ros2 run ai_sapiens_sim2real dualsense_teleop_ui_node "${UI_ARGS[@]}"
