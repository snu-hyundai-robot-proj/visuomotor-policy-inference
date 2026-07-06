#!/usr/bin/env bash
# Full DOWN -> UP recovery for one side's robot stack. Use when things get stuck.
# Handles the known gotchas seen in this project:
#   - a stray repo-side `vpi-inference` (hyphen) squatting on :8000
#   - teleop containers stuck in "Dead"/removing state (waits for the daemon)
#   - Inspire serial freeze (restarts the hand driver)
#   - Zivid camera disconnect (restarts the vision node)
#
#   ./restack_right.sh      # right side
#   ./restack_left.sh       # left side
#   ./restack.sh right      # (core, takes the side directly)
#
# NOT `set -e` — recovery must push past individual failures.
set -uo pipefail

SIDE="${1:-right}"
[ "$SIDE" = left ] || [ "$SIDE" = right ] || { echo "usage: $0 {left|right}"; exit 1; }
OTHER=$([ "$SIDE" = right ] && echo left || echo right)
TELEOP="$HOME/Tesollo/1.Project/1.hyundae/seoul_uiwang/system_Teleop"
[ "$SIDE" = right ] && HAND_TOPIC=/inspire/joint_states || HAND_TOPIC=/dg5f_left/joint_states

echo "================ restack: $SIDE ================"

# 1) drop the stray repo-side inference (hyphen) that holds :8000
docker rm -f vpi-inference >/dev/null 2>&1 && echo "[1] removed stray repo vpi-inference (freed :8000)" || echo "[1] no stray repo inference"

# 2) clean down of BOTH profiles
cd "$TELEOP" || { echo "ERROR: teleop dir not found: $TELEOP"; exit 1; }
echo "[2] docker compose down (both profiles) ..."
timeout 90 docker compose --profile left --profile right down 2>&1 | grep -iE 'removed|stopp' | tail -3

# 3) wait for stuck "Dead"/removing teleop containers to clear (daemon can lag ~30s)
echo -n "[3] waiting for cleanup "
for i in $(seq 1 30); do
  n=$(docker ps -a --format '{{.Names}}' | grep -cE 'teleop_(vision|hand)_(left|right)|^ros2_teleop_system$|^vpi_inference$')
  [ "${n:-0}" -eq 0 ] && { echo " clear"; break; }
  for c in ros2_teleop_system vpi_inference teleop_vision_left teleop_hand_left teleop_vision_right teleop_hand_right; do
    docker rm -f "$c" >/dev/null 2>&1
  done
  echo -n "."; sleep 2
done

# 4) bring the side up
echo "[4] docker compose --profile $SIDE up -d ..."
timeout 120 docker compose --profile "$SIDE" up -d 2>&1 | grep -iE 'started|created|error' | tail -6

# 5) wait for the inference model to load
echo -n "[5] inference: "
for i in $(seq 1 30); do
  curl -s --max-time 3 localhost:8000/info 2>/dev/null | grep -q "${SIDE}-flowmatch" && { echo "${SIDE} model ready"; break; }
  sleep 3; [ "$i" -eq 30 ] && echo "NOT ready (check: docker logs vpi_inference)"
done

# 6) reset the HAND only (serial freeze). Do NOT touch the vision node — restarting it
# re-triggers a slow/fragile Zivid reconnect. Leave vision alone (user request).
echo "[6] resetting hand driver (vision left untouched) ..."
docker restart "teleop_hand_${SIDE}" >/dev/null 2>&1
sleep 10

# 7) verify
echo "[7] status:"
docker ps --format '   {{.Names}}\t{{.Status}}' | grep -E 'teleop|vpi_inference|ros2_teleop' | sort
curl -s --max-time 4 localhost:8000/info 2>/dev/null | grep -o '"model_id":"[^"]*"' | sed 's/^/   model: /'
docker exec ros2_teleop_system bash -lc "source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=0;
  for t in /system_${SIDE}/zivid_rgb /system_${SIDE}/d405_rgb ${HAND_TOPIC}; do
    echo -n \"   \$t: \"; timeout 5 ros2 topic hz \$t 2>/dev/null | grep -m1 'average rate' || echo 'NO DATA';
  done" 2>/dev/null

echo "================ done ($SIDE) ================"
echo "if the hand still won't move:  docker restart teleop_hand_${SIDE}"
[ "$SIDE" = left ] && echo "NOTE(left): DG5F needs power + ethernet on 192.168.4.x, and ros2_control baked into the image."
