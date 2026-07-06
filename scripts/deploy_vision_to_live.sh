#!/usr/bin/env bash
# Deploy the repo's teleop_vision package (incl. the fusion subpackage) into the LIVE
# ROS workspace that the running container mounts, then rebuild it in-container.
#
# The live workspace is a SEPARATE checkout from this repo; we never hand-edit it —
# this script is the only way changes leave the repo. Run it from the repo root.
#
#   ./scripts/deploy_vision_to_live.sh                # rsync + colcon build
#   LIVE_WS=/path/to/system_Teleop ./scripts/deploy_vision_to_live.sh
#   CONTAINER=ros2_teleop_system ./scripts/deploy_vision_to_live.sh
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPO_VISION="$REPO_ROOT/system_Teleop/src/Vision_"
LIVE_WS="${LIVE_WS:-/home/bi/Tesollo/1.Project/1.hyundae/seoul_uiwang/system_Teleop}"
CONTAINER="${CONTAINER:-ros2_teleop_system}"
LIVE_VISION="$LIVE_WS/src/Vision_"

[ -d "$REPO_VISION" ] || { echo "repo Vision_ not found: $REPO_VISION" >&2; exit 1; }
[ -d "$LIVE_WS/src" ] || { echo "live workspace not found: $LIVE_WS/src" >&2; exit 1; }

echo "==> rsync $REPO_VISION  ->  $LIVE_VISION"
rsync -av --delete \
  --exclude '__pycache__/' --exclude '*.pyc' \
  --exclude 'data/' --exclude 'build/' --exclude 'install/' \
  "$REPO_VISION/" "$LIVE_VISION/"

echo "==> colcon build (teleop_vision) inside $CONTAINER"
docker exec "$CONTAINER" bash -lc '
  source /opt/ros/humble/setup.bash &&
  cd /workspace &&
  colcon build --packages-select teleop_vision --symlink-install
'

cat <<'EOF'

==> done. Next:
  # 1) (once) calibrate the wrist camera — D405 only, vision node may stay up:
  docker exec -it ros2_teleop_system bash -lc \
    'source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash &&
     ros2 run teleop_vision handeye_calibrate --ros-args -p side:=right \
       -p square_len_mm:=30.0 -p marker_len_mm:=22.0'
  # 2) restart the vision node so fusion loads, then trigger a fused capture:
  ros2 service call /system_right/fuse_cloud std_srvs/srv/Trigger {}
  # fused .ply -> Record/right/fused/ ; PointCloud2 -> /system_right/fused_cloud
EOF
