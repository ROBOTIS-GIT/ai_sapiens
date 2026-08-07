#!/usr/bin/env bash

set -e

JOY_PID=""

cleanup() {
  if [ -n "$JOY_PID" ] && kill -0 "$JOY_PID" 2>/dev/null; then
    kill "$JOY_PID"
    wait "$JOY_PID" 2>/dev/null || true
  fi
}

trap cleanup EXIT INT TERM

ros2 run joy joy_node --ros-args \
  -p device_id:=0 \
  -p deadzone:=0.0 \
  --log-level warn &
JOY_PID=$!

ros2 run ai_sapiens_sim2real dualsense_teleop_ui_node "$@"
