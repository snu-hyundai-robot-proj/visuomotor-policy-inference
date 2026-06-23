#!/usr/bin/env bash
set -euo pipefail

cd /workspace
source /opt/ros/humble/setup.bash

python3 -m pip install "setuptools==58.2.0"

if [ ! -f install/system_interface/share/system_interface/package.bash ] || \
   [ ! -f install/teleop_vision/share/teleop_vision/package.bash ]; then
  apt-get update
  rosdep install --from-paths src/System_/system_interface src/Vision_ --ignore-src -r -y
fi

colcon build --symlink-install --packages-up-to teleop_vision
source install/setup.bash
