#!/usr/bin/env bash
# Replay a recorded episode through the model onto the REAL robot, from the host.
# Feeds the episode's images + state to /predict and drives arm + hand with the output.
#
#   ./run_episode_replay.sh                # right side, all frames, arm + hand
#   ./run_episode_replay.sh right 0        # explicit (steps=0 -> all frames)
#   ./run_episode_replay.sh right 120      # only first 120 frames
#   ./run_episode_replay.sh left 0 --recorded   # use recorded action instead of the model
#   DRY=1 ./run_episode_replay.sh right    # DRY-RUN (compute only, no motion)
#
# Prereq: the side's stack must be up (model + hand driver):
#   cd <system_Teleop> && docker compose --profile <side> up -d
set -euo pipefail

SIDE="${1:-right}"
STEPS="${2:-0}"                            # 0 = all frames
shift 2>/dev/null || true; shift 2>/dev/null || true
PASS="$*"                                  # extra flags, e.g. --recorded

CONTAINER="ros2_teleop_system"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENGAGE="--engage"; [ -n "${DRY:-}" ] && ENGAGE=""
SRC="model"; echo "$PASS" | grep -q -- "--recorded" && { SRC="recorded"; PASS="${PASS/--recorded/}"; }

echo "[run_episode_replay] side=$SIDE steps=${STEPS:-all} source=$SRC ${ENGAGE:-DRY-RUN}"

# 0) sanity: container + inference server
if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER}$"; then
  echo "ERROR: container '$CONTAINER' not running -> docker compose --profile $SIDE up -d"; exit 1
fi
if ! curl -s --max-time 5 http://localhost:8000/info | grep -q flowmatch; then
  echo "ERROR: inference server not ready on :8000"; exit 1
fi

# 1) robot-network IP on enp4s0 (non-persistent)
sudo ip addr add 192.168.4.55/24 dev enp4s0 2>/dev/null || true

# 2) copy the latest replay script + the episode into the container
docker cp "$REPO/examples/drive_arm_hand_replay.py" "$CONTAINER:/tmp/drive_arm_hand_replay.py" >/dev/null
docker exec "$CONTAINER" bash -lc "ls /tmp/sample_episodes/$SIDE/episode.json >/dev/null 2>&1" \
  || docker cp "$REPO/examples/sample_episodes" "$CONTAINER:/tmp/sample_episodes" >/dev/null

# 3) run the replay.
#    Ctrl-C on the host does not reliably reach the python inside `docker exec`; this trap
#    forwards the interrupt so the script stops cleanly (arm.stop) instead of finishing the run.
trap 'echo; echo "[stop] interrupting replay ..."; docker exec "$CONTAINER" pkill -INT -f drive_arm_hand_replay 2>/dev/null; sleep 1' INT TERM

docker exec "$CONTAINER" bash -lc \
  "source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=0; cd /tmp; \
   python3 -u drive_arm_hand_replay.py --side $SIDE --source $SRC --steps $STEPS $ENGAGE $PASS"
