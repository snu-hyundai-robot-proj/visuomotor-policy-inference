#!/usr/bin/env bash
# Run the live closed-loop policy on the REAL robot, from the host (outside docker).
#
#   ./run_closed_loop.sh                 # right side, 60 s, arm + hand
#   ./run_closed_loop.sh right 60        # explicit
#   ./run_closed_loop.sh left 60 --no-hand   # left, arm only (DG5F off)
#   ./run_closed_loop.sh right 60 --hand-only  # arm off, hand only
#   DRY=1 ./run_closed_loop.sh right     # DRY-RUN (predict only, no motion)
#
# Prereq: the side's stack must be up first:
#   cd <system_Teleop> && docker compose --profile <side> up -d
# (vision node + hand driver + inference model for that side)
set -euo pipefail

SIDE="${1:-right}"
SECS="${2:-20}"
shift 2>/dev/null || true; shift 2>/dev/null || true
PASS="$*"                                  # extra flags, e.g. --no-hand / --hand-only

CONTAINER="ros2_teleop_system"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENGAGE="--engage"; [ -n "${DRY:-}" ] && ENGAGE=""   # DRY=1 -> no motion

echo "[run_closed_loop] side=$SIDE secs=$SECS ${ENGAGE:-DRY-RUN} ${PASS}"

# 0) sanity: container + inference server
if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER}$"; then
  echo "ERROR: container '$CONTAINER' not running. Start the stack:"
  echo "       docker compose --profile $SIDE up -d"; exit 1
fi
if ! curl -s --max-time 5 http://localhost:8000/info | grep -q flowmatch; then
  echo "ERROR: inference server not ready on :8000 (is the $SIDE model loaded?)"; exit 1
fi

# 1) robot-network IP on enp4s0 (non-persistent — re-add each time)
sudo ip addr add 192.168.4.55/24 dev enp4s0 2>/dev/null || true

# 2) copy the latest loop script into the container
docker cp "$REPO/examples/run_robot_loop.py" "$CONTAINER:/tmp/run_robot_loop.py" >/dev/null

# 3) run the closed loop
docker exec "$CONTAINER" bash -lc \
  "source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=0; cd /tmp; \
   python3 run_robot_loop.py --side $SIDE $ENGAGE --secs $SECS $PASS"
