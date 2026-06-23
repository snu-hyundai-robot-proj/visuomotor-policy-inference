#!/usr/bin/env bash
# Bring a side's FULL robot stack UP: vision + hand + inference. Cleans first (Zivid is
# exclusive, so the other side is taken down), then resets the hand + vision drivers
# (serial freeze / Zivid disconnect are common right after a fresh up).
#
#   ./up.sh right
#   ./up.sh left
set -uo pipefail

SIDE="${1:-right}"
[ "$SIDE" = left ] || [ "$SIDE" = right ] || { echo "usage: $0 {left|right}"; exit 1; }
TELEOP="$HOME/Tesollo/1.Project/1.hyundae/seoul_uiwang/system_Teleop"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
[ "$SIDE" = right ] && HAND_TOPIC=/inspire/joint_states || HAND_TOPIC=/dg5f_left/joint_states

echo "================ up: $SIDE ================"

# 1) clean slate (also frees Zivid from the other side)
"$REPO/down.sh"

# 2) up the requested side
cd "$TELEOP" || { echo "ERROR: teleop dir not found: $TELEOP"; exit 1; }
echo "[*] docker compose --profile $SIDE up -d ..."
timeout 120 docker compose --profile "$SIDE" up -d 2>&1 | grep -iE 'started|created|error' | tail -6

# 3) wait for the inference model to load
echo -n "[*] inference: "
for i in $(seq 1 30); do
  curl -s --max-time 3 localhost:8000/info 2>/dev/null | grep -q "${SIDE}-flowmatch" && { echo "${SIDE} model ready"; break; }
  sleep 3; [ "$i" -eq 30 ] && echo "NOT ready (check: docker logs vpi_inference)"
done

# 4) reset hand (serial freeze) + vision (zivid disconnect) — common after a fresh up
echo "[*] resetting hand + vision driver ..."
docker restart "teleop_hand_${SIDE}" "teleop_vision_${SIDE}" >/dev/null 2>&1
sleep 10

# 5) verify
echo "[*] status:"
docker ps --format '   {{.Names}}\t{{.Status}}' | grep -E 'teleop|vpi_inference|ros2_teleop' | sort
curl -s --max-time 4 localhost:8000/info 2>/dev/null | grep -o '"model_id":"[^"]*"' | sed 's/^/   model: /'
docker exec ros2_teleop_system bash -lc "source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=0;
  for t in /system_${SIDE}/zivid_rgb /system_${SIDE}/d405_rgb ${HAND_TOPIC}; do
    echo -n \"   \$t: \"; timeout 5 ros2 topic hz \$t 2>/dev/null | grep -m1 'average rate' || echo 'NO DATA';
  done" 2>/dev/null

echo "================ up done ($SIDE) ================"
echo "if the hand still won't move:  docker restart teleop_hand_${SIDE}"
[ "$SIDE" = left ] && echo "NOTE(left): DG5F needs power + ethernet on 192.168.4.x, and ros2_control baked into the image."
