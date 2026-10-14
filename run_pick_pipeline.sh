#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Default ports. Override via env vars if needed.
ULTRASONIC_PORT="${ULTRASONIC_PORT:-/dev/ttyUSB0}"
TRACKER_PORT="${TRACKER_PORT:-/dev/propeller}"
ARM_PORT="${ARM_PORT:-/dev/ttyACM0}"

CAMERA_DEVICE="${CAMERA_DEVICE:-/dev/usbcam}"
CAMERA_INDEX="${CAMERA_INDEX:-0}"
STOP_DISTANCE_CM="${STOP_DISTANCE_CM:-15.0}"

if [[ ! -f "$ROOT_DIR/install/setup.bash" ]]; then
  echo "[ERROR] Missing install/setup.bash. Build workspace once first:"
  echo "        colcon build --symlink-install"
  exit 1
fi

# Colcon setup scripts may reference optional vars like COLCON_TRACE.
# Temporarily disable nounset while sourcing to avoid unbound-variable exits.
set +u
source "$ROOT_DIR/install/setup.bash"
set -u

if ! command -v ros2 >/dev/null 2>&1; then
  echo "[ERROR] ros2 command not found after sourcing install/setup.bash"
  exit 1
fi

read -r -p "Enter target object (bottle/cup/sports ball): " TARGET_OBJECT
TARGET_OBJECT="${TARGET_OBJECT#${TARGET_OBJECT%%[![:space:]]*}}"
TARGET_OBJECT="${TARGET_OBJECT%${TARGET_OBJECT##*[![:space:]]}}"

case "$TARGET_OBJECT" in
  bottle|cup|"sports ball")
    ;;
  *)
    echo "[ERROR] Unsupported target '$TARGET_OBJECT'."
    echo "        Use one of: bottle, cup, sports ball"
    exit 1
    ;;
esac

PIDS=()

cleanup() {
  echo
  echo "[INFO] Stopping all launched processes..."
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "$pid" >/dev/null 2>&1; then
      kill "$pid" >/dev/null 2>&1 || true
    fi
  done
  wait || true
  echo "[INFO] Cleanup done."
}
trap cleanup EXIT INT TERM

DET_ARGS=(--ros-args -p camera_index:="$CAMERA_INDEX")
if [[ -e "$CAMERA_DEVICE" ]]; then
  echo "[INFO] Starting object detector (camera_device=$CAMERA_DEVICE)..."
  DET_ARGS+=( -p camera_device:="$CAMERA_DEVICE" )
else
  echo "[WARN] CAMERA_DEVICE '$CAMERA_DEVICE' not found. Falling back to camera_index=$CAMERA_INDEX"
fi
ros2 run robo_vision object_det_remote "${DET_ARGS[@]}" &
PIDS+=("$!")

sleep 1

# Publish target once so detector/tracker pipeline starts with selected object.
echo "[INFO] Sending target command: $TARGET_OBJECT"
ros2 topic pub --once \
  --qos-reliability reliable \
  --qos-durability transient_local \
  /object_targets std_msgs/msg/String "{data: '$TARGET_OBJECT'}" >/dev/null

echo "[INFO] Starting chassis tracker (PID, stop_distance_cm=$STOP_DISTANCE_CM)..."
ros2 run robo_control propeller_tracker --ros-args \
  -p port:="$TRACKER_PORT" \
  -p stop_distance_cm:="$STOP_DISTANCE_CM" &
PIDS+=("$!")

# Optional ultrasonic publisher if sensor Arduino is on a dedicated port.
if [[ "$ULTRASONIC_PORT" != "$TRACKER_PORT" ]]; then
  echo "[INFO] Starting ultrasonic publisher on $ULTRASONIC_PORT..."
  # Override port by env var without changing node code via symlinked tty path if needed.
  ROS_ULTRASONIC_PORT="$ULTRASONIC_PORT" \
  ros2 run brain arduino_ping &
  PIDS+=("$!")
else
  echo "[WARN] ULTRASONIC_PORT and TRACKER_PORT are both '$TRACKER_PORT'."
  echo "[WARN] Skipping arduino_ping to avoid serial conflict on one USB port."
fi

# Arm mapper listens to ultrasonic_data and publishes predefined joint steps.
echo "[INFO] Starting ultrasonic->arm mapper..."
ros2 run robo_control ultrasonic_arm_mapper &
PIDS+=("$!")

# Step bridge sends joint steps to arm controller board.
if [[ -e "$ARM_PORT" ]]; then
  echo "[INFO] Starting arm step bridge on $ARM_PORT..."
  ros2 run robo_control step_bridge_arm -- --port "$ARM_PORT" --baud 115200 --topic joint_steps_abs &
  PIDS+=("$!")
else
  echo "[WARN] ARM_PORT '$ARM_PORT' not found. Arm serial bridge not started."
fi

echo "[INFO] Pipeline running. Press Ctrl+C to stop all processes."
wait
