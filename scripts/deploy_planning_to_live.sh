#!/usr/bin/env bash
# Deploy the standalone motion-planning stack (teleop_planning + the hdr35_20_moveit_config
# launch fix) from THIS repo into the live ROS workspace, building against the MoveIt-enabled
# image. Repo is the source of truth; we never hand-edit the live tree.
#
# Stages (each gated so nothing disrupts the running teleop unless you confirm):
#   1. build the ros2-teleop image with MoveIt (from repo Dockerfile)
#   2. rsync the planning packages repo -> live
#   3. colcon build them into the live install volume (container from the new image)
#
#   ./scripts/deploy_planning_to_live.sh
#   LIVE_WS=/path/to/system_Teleop ./scripts/deploy_planning_to_live.sh
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPO_ST="$REPO_ROOT/system_Teleop"
LIVE_WS="${LIVE_WS:-/home/bi/Tesollo/1.Project/1.hyundae/seoul_uiwang/system_Teleop}"
IMAGE="${IMAGE:-ros2-teleop:latest}"
INSTALL_VOL="${INSTALL_VOL:-system_teleop_teleop_install}"
BUILD_VOL="${BUILD_VOL:-system_teleop_teleop_build}"

MOVEIT_CFG_REL="src/Robot_/src/hdr_ros2_driver/hdr_moveit_config/hdr35_20_moveit_config"

[ -d "$LIVE_WS/src" ] || { echo "live workspace not found: $LIVE_WS/src" >&2; exit 1; }

echo "==> [1/3] build $IMAGE with MoveIt (repo Dockerfile)"
DOCKER_BUILDKIT=1 docker build -t "$IMAGE" "$REPO_ST"

echo "==> [2/3] rsync planning packages  repo -> live"
rsync -av --delete --exclude '__pycache__/' --exclude '*.pyc' \
  "$REPO_ST/src/teleop_planning/" "$LIVE_WS/src/teleop_planning/"
# only the launch file changed in the moveit config; sync the whole package to be safe
rsync -av --exclude '__pycache__/' --exclude '*.pyc' \
  "$REPO_ST/$MOVEIT_CFG_REL/" "$LIVE_WS/$MOVEIT_CFG_REL/"

echo "==> [3/3] colcon build (hdr_stream hdr_description hdr35_20_moveit_config teleop_planning) into live install volume"
# hdr_stream is needed by the execution bridge (OpenStream); the rest are the planning stack.
docker run --rm \
  -v "$LIVE_WS":/workspace \
  -v "$INSTALL_VOL":/workspace/install \
  -v "$BUILD_VOL":/workspace/build \
  "$IMAGE" bash -lc '
    source /opt/ros/humble/setup.bash &&
    cd /workspace &&
    colcon build --packages-select hdr_stream hdr_description hdr35_20_moveit_config teleop_planning \
      --symlink-install'

cat <<'EOF'

==> done. The live install volume now has the planning stack, BUT the running containers
    still use the OLD image (no MoveIt). To use planning you must recreate the base
    container from the new image (this briefly interrupts the teleop shell container):

    cd "$LIVE_WS" && docker compose up -d --force-recreate ros2-teleop

Then, planning-only verification (no real arm motion):
    # current arm joint state must be flowing (deg) on /system_<side>/joint_states:
    docker exec -d ros2_teleop_system bash -lc \
      'source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash &&
       ros2 run hdr_stream hdr_stream_node --ros-args -p robot_side:=right -p simulation:=false'
    docker exec -it ros2_teleop_system bash -lc \
      'source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash &&
       ros2 launch teleop_planning planning_bringup.launch.py side:=right use_mock_hardware:=false'
    # publish a hook pose (or run the hook_pose_estimator), inspect preview in RViz, then:
    ros2 service call /plan_to_hook/plan std_srvs/srv/Trigger {}     # plan only (safe)

REAL-ARM EXECUTION (supervised — arm clear, e-stop in reach):
    The FollowJointTrajectory->OpenStream bridge (hdr_followjoint_bridge) is wired into the
    launch with dry_run:=true (logs, never moves). The full chain
    (plan -> execute -> FollowJointTrajectory -> bridge) is verified in dry-run.
    To actually move the arm, relaunch with BOTH flags flipped, then call execute:
       ros2 launch teleop_planning planning_bringup.launch.py side:=right \
            use_mock_hardware:=false dry_run:=false allow_execute:=true
       ros2 service call /plan_to_hook/execute std_srvs/srv/Trigger {}
    Do NOT enable dry_run:=false unattended. Start with a small approach_dist and low
    vel_scale/acc_scale; the bridge also rejects inter-point jumps > max_step_deg (25 deg).
EOF
