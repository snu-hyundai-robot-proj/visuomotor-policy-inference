#!/usr/bin/env bash
set -euo pipefail

SIDE="${1:-right}"
CONTAINER="${CONTAINER:-ros2_teleop_system}"
SERVICE_NAME="${SERVICE_NAME:-hook_detect_object}"
POSE_TOPIC="${POSE_TOPIC:-hook_pose}"
OUTPUT_DIR="${OUTPUT_DIR:-}"
SAVE_DEBUG_IMAGES="${SAVE_DEBUG_IMAGES:-false}"
CONFIDENCE_THRESHOLD="${CONFIDENCE_THRESHOLD:-0.8}"
SERVICE_TIMEOUT="${SERVICE_TIMEOUT:-180s}"
BUILD="${BUILD:-1}"

if [[ "$SIDE" != "left" && "$SIDE" != "right" ]]; then
  echo "Usage: $0 [left|right]"
  exit 2
fi

if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
  echo "Container '$CONTAINER' is not running."
  echo "Start it first, for example: cd system_Teleop && docker compose up -d ros2-teleop"
  exit 1
fi

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
local_data_dir="$repo_root/Vision_/data"
workspace_src="$(docker inspect "$CONTAINER" \
  --format '{{range .Mounts}}{{if eq .Destination "/workspace"}}{{.Source}}{{end}}{{end}}')"

if [[ -z "$workspace_src" || ! -d "$workspace_src/src/Vision_" ]]; then
  echo "Could not find the host path mounted at /workspace/src/Vision_ for container '$CONTAINER'."
  exit 1
fi

echo "[host] Syncing hook pose estimator into mounted workspace:"
echo "       $workspace_src/src/Vision_"
cp "$repo_root/Vision_/teleop_vision/hook_pose_estimator.py" \
   "$workspace_src/src/Vision_/teleop_vision/hook_pose_estimator.py"

mkdir -p "$workspace_src/src/Vision_/segmentation/weights"
if [[ -f "$repo_root/Vision_/segmentation/weights/rf_detr_best.pth" ]]; then
  cp "$repo_root/Vision_/segmentation/weights/rf_detr_best.pth" \
     "$workspace_src/src/Vision_/segmentation/weights/rf_detr_best.pth"
fi

python3 - "$workspace_src/src/Vision_" <<'PY'
from pathlib import Path
import sys

root = Path(sys.argv[1])
setup = root / "setup.py"
text = setup.read_text()
text = text.replace(
    "# 'hook_pose_estimator = teleop_vision.hook_pose_estimator:main',",
    "'hook_pose_estimator = teleop_vision.hook_pose_estimator:main',",
)
setup.write_text(text)

package = root / "package.xml"
text = package.read_text()
if "<depend>geometry_msgs</depend>" not in text:
    text = text.replace(
        "  <depend>cv_bridge</depend>\n",
        "  <depend>cv_bridge</depend>\n"
        "  <depend>geometry_msgs</depend>\n"
        "  <depend>std_srvs</depend>\n",
    )
package.write_text(text)
PY

docker exec \
  -e SIDE="$SIDE" \
  -e SERVICE_NAME="$SERVICE_NAME" \
  -e POSE_TOPIC="$POSE_TOPIC" \
  -e OUTPUT_DIR="$OUTPUT_DIR" \
  -e SAVE_DEBUG_IMAGES="$SAVE_DEBUG_IMAGES" \
  -e CONFIDENCE_THRESHOLD="$CONFIDENCE_THRESHOLD" \
  -e SERVICE_TIMEOUT="$SERVICE_TIMEOUT" \
  -e BUILD="$BUILD" \
  "$CONTAINER" bash -lc '
set -euo pipefail
set +u
source /opt/ros/humble/setup.bash
set -u

cd /workspace
if [[ "$BUILD" == "1" ]]; then
  echo "[container] Building teleop_vision..."
  colcon build --packages-select teleop_vision --event-handlers console_direct+
fi

set +u
source /workspace/install/setup.bash
set -u

cleanup_old_hook_nodes() {
python3 - <<'PY'
import os
import signal

target = "/workspace/install/teleop_vision/lib/teleop_vision/hook_pose_estimator"

for name in os.listdir("/proc"):
    if not name.isdigit():
        continue
    pid = int(name)
    if pid in (os.getpid(), os.getppid()):
        continue
    try:
        args = [
            item.decode(errors="ignore")
            for item in open(f"/proc/{pid}/cmdline", "rb").read().split(b"\0")
            if item
        ]
    except OSError:
        continue
    if target in args:
        try:
            print(f"[container] killing stale hook_pose_estimator pid={pid}")
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
PY
}

echo "[container] Cleaning up any previous hook_pose_estimator processes..."
cleanup_old_hook_nodes
sleep 1

service="/${SERVICE_NAME}"
topic="/${POSE_TOPIC}"
out_arg=()
if [[ -n "${OUTPUT_DIR}" ]]; then
  mkdir -p "${OUTPUT_DIR}"
  out_arg=(-p "output_dir:=${OUTPUT_DIR}")
fi

echo "[container] Starting hook_pose_estimator for side=${SIDE}"
setsid ros2 run teleop_vision hook_pose_estimator --ros-args \
  -p "hand_side:=${SIDE}" \
  -p "service_name:=${SERVICE_NAME}" \
  -p "pose_topic:=${POSE_TOPIC}" \
  -p "save_debug_images:=${SAVE_DEBUG_IMAGES}" \
  -p "confidence_threshold:=${CONFIDENCE_THRESHOLD}" \
  "${out_arg[@]}" &
node_pid=$!

cleanup() {
  kill -- "-$node_pid" >/dev/null 2>&1 || true
  kill "$node_pid" >/dev/null 2>&1 || true
  wait "$node_pid" >/dev/null 2>&1 || true
  cleanup_old_hook_nodes >/dev/null 2>&1 || true
}
trap cleanup EXIT

echo "[container] Waiting for service ${service}..."
for _ in $(seq 1 120); do
  if ros2 service list | grep -qx "$service"; then
    break
  fi
  if ! kill -0 "$node_pid" >/dev/null 2>&1; then
    echo "[container] hook_pose_estimator exited before service became available."
    wait "$node_pid"
  fi
  sleep 1
done

if ! ros2 service list | grep -qx "$service"; then
  echo "[container] Timed out waiting for ${service}."
  exit 1
fi

srv_type="$(ros2 service type "$service")"
echo "[container] Calling ${service} (${srv_type})..."
if [[ "$srv_type" == "system_interface/srv/DetectObject" ]]; then
  timeout "$SERVICE_TIMEOUT" ros2 service call "$service" system_interface/srv/DetectObject "{}"
else
  timeout "$SERVICE_TIMEOUT" ros2 service call "$service" std_srvs/srv/Trigger "{}"
fi

echo "[container] Waiting for one pose on ${topic}..."
timeout 20s ros2 topic echo --once "$topic" || true

echo "[container] Recent output files:"
result_dir="${OUTPUT_DIR:-/workspace/src/Vision_/data}"
find "$result_dir" -maxdepth 1 -type f -printf "%TY-%Tm-%Td %TH:%TM %p\n" 2>/dev/null \
  | sort \
  | tail -20 || true

python3 - "$result_dir" "$SIDE" <<'PY'
from pathlib import Path
import sys

import numpy as np

result_dir = Path(sys.argv[1])
side = sys.argv[2]
files = sorted(result_dir.glob(f"{side}_*_data.bin"), key=lambda path: path.stat().st_mtime)
if not files:
    print("[container] No pose data bin found.")
    raise SystemExit(0)

path = files[-1]
pose = np.fromfile(path, dtype=np.float32)
print(f"[container] Latest pose bin: {path}")
print("[container] pose_7d_vec [x, y, z, qx, qy, qz, qw]:")
print(" ".join(f"{value:.6f}" for value in pose.tolist()))
PY
'

mkdir -p "$local_data_dir"
remote_data_dir="$workspace_src/src/Vision_/data"
if [[ -d "$remote_data_dir" ]]; then
  echo "[host] Copying latest ${SIDE} results into $local_data_dir"
  find "$remote_data_dir" -maxdepth 1 -type f -name "${SIDE}_*" -print0 \
    | xargs -0 -r cp -t "$local_data_dir"

  echo "[host] Latest local result files:"
  find "$local_data_dir" -maxdepth 1 -type f -name "${SIDE}_*" -printf "%TY-%Tm-%Td %TH:%TM %p\n" \
    | sort \
    | tail -20
fi
