#!/usr/bin/env bash
# Launch the DG5F LEFT per-joint control GUI (all 20 DOF, each with a slider + live numbers).
# Runs in a fresh teleop container (PyQt5 + rclpy + host net + X11 + shared /dev/shm for DDS).
#
#   ./hand_joint_gui.sh
#
# Prereq: left hand stack up (./up.sh left) so the DG5F controller is subscribed.
set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DISPLAY="${DISPLAY:-:0}"
sudo -u pin DISPLAY="$DISPLAY" XAUTHORITY=/home/pin/.Xauthority xhost +local: >/dev/null 2>&1 \
  || xhost +local: >/dev/null 2>&1 || true

docker rm -f hand_joint_gui >/dev/null 2>&1 || true
echo "[hand_joint_gui] launching (DG5F left, 20 DOF) ..."
docker run --rm -it --name hand_joint_gui \
  --network host \
  -e DISPLAY="$DISPLAY" -e ROS_DOMAIN_ID=0 -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /dev/shm:/dev/shm:rw \
  -v "$REPO/examples/hand_joint_gui.py:/tmp/hand_joint_gui.py:ro" \
  ros2-teleop:latest \
  bash -lc "source /opt/ros/humble/setup.bash; python3 -u /tmp/hand_joint_gui.py"
