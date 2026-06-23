#!/usr/bin/env bash
# Send the robot back to its mean INIT pose ("제자리"), from the host. Arm + hand, side-aware.
#
#   ./home.sh                 # right side, arm + hand
#   ./home.sh right
#   ./home.sh left            # left (arm + DG5F hand)
#   ./home.sh left --hand-only   # only the hand (e.g. arm powered off)
#   DRY=1 ./home.sh right     # dry-run — shows the gap to init, NO motion
#
# Prereq: that side's stack up (hand driver + arm reachable).
# Stop midway: Ctrl-C (forwarded into the container) or
#   docker exec ros2_teleop_system pkill -INT -f home_to_init
set -uo pipefail

SIDE="${1:-right}"
[ "$SIDE" = left ] || [ "$SIDE" = right ] || { echo "usage: $0 {left|right} [extra flags]"; exit 1; }
shift 2>/dev/null || true
PASS="$*"                                   # extra flags, e.g. --hand-only / --arm-speed 12

CONTAINER="ros2_teleop_system"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENGAGE="--engage"; [ -n "${DRY:-}" ] && ENGAGE=""   # DRY=1 -> no motion

echo "[home] side=$SIDE ${ENGAGE:-DRY-RUN} $PASS"

docker ps --format '{{.Names}}' | grep -q "^${CONTAINER}$" \
  || { echo "ERROR: '$CONTAINER' not running -> docker compose --profile $SIDE up -d"; exit 1; }

# robot-network IP on enp4s0 (non-persistent)
sudo ip addr add 192.168.4.55/24 dev enp4s0 2>/dev/null || true

docker cp "$REPO/examples/home_to_init.py" "$CONTAINER:/tmp/home_to_init.py" >/dev/null

# forward Ctrl-C into the container so it stops cleanly (arm.stop)
trap 'echo; echo "[stop] interrupting home ..."; docker exec "$CONTAINER" pkill -INT -f home_to_init 2>/dev/null; sleep 1' INT TERM

docker exec "$CONTAINER" bash -lc \
  "source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=0; cd /tmp; \
   python3 -u home_to_init.py --side $SIDE $ENGAGE $PASS"
