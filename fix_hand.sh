#!/usr/bin/env bash
# Fix a stuck/frozen hand. Restarts the driver, then VERIFIES the hand actually moves
# (grip -> open, checks the joint state changed). Retries until it works.
#   ./fix_hand.sh right    # Inspire
#   ./fix_hand.sh left     # DG5F
#
# The Inspire serial connection sometimes freezes (joint_states goes stale, commands ignored).
# A single restart usually fixes it, but the first start can come up reading 0 — so this loops.
set -uo pipefail

SIDE="${1:-right}"
[ "$SIDE" = left ] || [ "$SIDE" = right ] || { echo "usage: $0 {left|right}"; exit 1; }
CONTAINER="ros2_teleop_system"
HAND="teleop_hand_${SIDE}"
MAX=4

if [ "$SIDE" = right ]; then
  TOPIC=/inspire/right/target;  TYPE=std_msgs/msg/Float64MultiArray;  STATE=/inspire/joint_states
  GRIP='{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}'      # Inspire: low = close
  OPEN='{data: [0.9, 0.9, 0.9, 0.9, 0.6, 0.6]}'
else
  TOPIC=/dg5f_left/lj_dg_pospid/reference;  TYPE=control_msgs/msg/MultiDOFCommand;  STATE=/dg5f_left/joint_states
  N='[lj_dg_1_1,lj_dg_1_2,lj_dg_1_3,lj_dg_1_4,lj_dg_2_1,lj_dg_2_2,lj_dg_2_3,lj_dg_2_4,lj_dg_3_1,lj_dg_3_2,lj_dg_3_3,lj_dg_3_4,lj_dg_4_1,lj_dg_4_2,lj_dg_4_3,lj_dg_4_4,lj_dg_5_1,lj_dg_5_2,lj_dg_5_3,lj_dg_5_4]'
  Z='[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]'
  G='[0,0.5,1.0,0.6, 0,0.5,1.0,0.6, 0,0.5,1.0,0.6, 0,0.5,1.0,0.6, 0,0.5,1.0,0.6]'
  GRIP="{dof_names: $N, values: $G, values_dot: $Z}"
  OPEN="{dof_names: $N, values: $Z, values_dot: $Z}"
fi

docker ps --format '{{.Names}}' | grep -q "^${CONTAINER}$" \
  || { echo "ERROR: '$CONTAINER' not running -> ./up.sh $SIDE"; exit 1; }

# returns 0 (success) if grip vs open joint states differ -> the hand physically moved
test_move() {
  docker exec -e TOPIC="$TOPIC" -e TYPE="$TYPE" -e STATE="$STATE" -e GRIP="$GRIP" -e OPEN="$OPEN" \
    "$CONTAINER" bash -lc '
    source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=0
    read6() { timeout 4 ros2 topic echo --once "$STATE" 2>/dev/null | sed -n "/position:/,/]/p" | grep -E "^- " | head -6 | sed "s/^- *//" | tr "\n" " "; }
    timeout 8 ros2 topic pub --once -w 1 "$TOPIC" "$TYPE" "$GRIP" >/dev/null 2>&1; sleep 2; A="$(read6)"
    timeout 8 ros2 topic pub --once -w 1 "$TOPIC" "$TYPE" "$OPEN" >/dev/null 2>&1; sleep 2; B="$(read6)"
    python3 -c "
import sys
a=[float(x) for x in \"$A\".split()]; b=[float(x) for x in \"$B\".split()]
d=max((abs(x-y) for x,y in zip(a,b)), default=0)
st = \"MOVES\" if d>15 else \"STUCK\"
print(\"   grip=\", [round(x,1) for x in a])
print(\"   open=\", [round(x,1) for x in b])
print(\"   max move=%.1f  -> %s\" % (d, st))
sys.exit(0 if d>15 else 1)
"'
}

echo "[fix_hand] side=$SIDE  (driver: $HAND)"
for a in $(seq 1 $MAX); do
  echo ">> attempt $a/$MAX: docker restart $HAND ..."
  docker restart "$HAND" >/dev/null 2>&1
  sleep 8
  if test_move; then
    echo "[fix_hand] ✅ FIXED — hand moves (grip->open verified)."
    exit 0
  fi
  echo "   still stuck, retrying ..."
done

echo "[fix_hand] ❌ FAILED after $MAX attempts."
echo "  - check driver log:  docker logs $HAND | tail"
[ "$SIDE" = right ] && echo "  - serial port moved? ls -l /dev/serial/by-id/   (Inspire = FT232R / BG024HL9)"
[ "$SIDE" = left ]  && echo "  - DG5F powered + ethernet on 192.168.4.x? ros2_control in image?"
exit 1
