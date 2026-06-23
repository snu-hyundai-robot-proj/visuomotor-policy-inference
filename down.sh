#!/usr/bin/env bash
# Take the WHOLE robot stack DOWN (both sides — no left/right distinction).
# Cleans the known stuck states: stray repo-side inference on :8000, and teleop
# containers stuck in "Dead"/removing state.
#
#   ./down.sh
set -uo pipefail   # not -e: must push past individual failures

TELEOP="$HOME/Tesollo/1.Project/1.hyundae/seoul_uiwang/system_Teleop"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "================ down (all) ================"

# stray repo-side inference (hyphen) that squats :8000
docker rm -f vpi-inference >/dev/null 2>&1 && echo "[*] removed stray repo vpi-inference (freed :8000)"

cd "$TELEOP" || { echo "ERROR: teleop dir not found: $TELEOP"; exit 1; }
echo "[*] docker compose down (both profiles) ..."
timeout 90 docker compose --profile left --profile right down 2>&1 | grep -iE 'removed|stopp' | tail -4

# force-clear any container stuck in Dead/removing (daemon can lag ~30s)
echo -n "[*] cleanup "
for i in $(seq 1 30); do
  n=$(docker ps -a --format '{{.Names}}' | grep -cE 'teleop_(vision|hand)_(left|right)|^ros2_teleop_system$|^vpi_inference$')
  [ "${n:-0}" -eq 0 ] && { echo "clear"; break; }
  for c in ros2_teleop_system vpi_inference teleop_vision_left teleop_hand_left teleop_vision_right teleop_hand_right; do
    docker rm -f "$c" >/dev/null 2>&1
  done
  echo -n "."; sleep 2
done

# repo web/visualization services (frontend, foxglove, web-video, rosbridge)
echo "[*] stopping web/visualization services ..."
( cd "$REPO" && timeout 60 docker compose -f docker-compose.full.yml down 2>&1 | grep -iE 'removed|stopp' | tail -4 )
# fallback: force-remove by name in case compose didn't manage them
docker rm -f vpi-frontend vpi-foxglove vpi-web-video vpi-rosbridge >/dev/null 2>&1

# leftover hello-world test containers
hw=$(docker ps -aq --filter ancestor=hello-world 2>/dev/null)
[ -n "$hw" ] && { docker rm -f $hw >/dev/null 2>&1; echo "[*] removed leftover hello-world containers"; }

echo "================ all down ================"
docker ps --format '{{.Names}}' | grep -qE 'teleop|vpi_inference|ros2_teleop' \
  && { echo "(still up — re-run, daemon may still be removing)"; docker ps --format '  {{.Names}}\t{{.Status}}' | grep -E 'teleop|vpi_inference|ros2_teleop'; } \
  || echo "(nothing running)"
