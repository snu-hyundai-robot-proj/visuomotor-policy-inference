#!/usr/bin/env bash
# Quick hand test: GRIP then OPEN. Nothing else.
#   ./test_hand.sh right    # Inspire (/inspire/right/target)
#   ./test_hand.sh left     # DG5F   (/dg5f_left/lj_dg_pospid/reference)
#
# Prereq: that side's hand driver up (./up.sh <side>). If the hand doesn't move,
# it's a serial freeze -> docker restart teleop_hand_<side>, then re-run.
set -uo pipefail

SIDE="${1:-right}"
[ "$SIDE" = left ] || [ "$SIDE" = right ] || { echo "usage: $0 {left|right}"; exit 1; }
CONTAINER="ros2_teleop_system"

if [ "$SIDE" = right ]; then
  TOPIC=/inspire/right/target
  TYPE=std_msgs/msg/Float64MultiArray
  STATE=/inspire/joint_states
  # Inspire: low normalized -> fingers CLOSED (grip), high -> OPEN (verified on hardware)
  GRIP='{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}'
  OPEN='{data: [0.9, 0.9, 0.9, 0.9, 0.6, 0.6]}'
else
  TOPIC=/dg5f_left/lj_dg_pospid/reference
  TYPE=control_msgs/msg/MultiDOFCommand
  STATE=/dg5f_left/joint_states
  N='[lj_dg_1_1,lj_dg_1_2,lj_dg_1_3,lj_dg_1_4,lj_dg_2_1,lj_dg_2_2,lj_dg_2_3,lj_dg_2_4,lj_dg_3_1,lj_dg_3_2,lj_dg_3_3,lj_dg_3_4,lj_dg_4_1,lj_dg_4_2,lj_dg_4_3,lj_dg_4_4,lj_dg_5_1,lj_dg_5_2,lj_dg_5_3,lj_dg_5_4]'
  Z='[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]'
  G='[0,0.5,1.0,0.6, 0,0.5,1.0,0.6, 0,0.5,1.0,0.6, 0,0.5,1.0,0.6, 0,0.5,1.0,0.6]'
  GRIP="{dof_names: $N, values: $G, values_dot: $Z}"
  OPEN="{dof_names: $N, values: $Z, values_dot: $Z}"
fi

echo "[test_hand] side=$SIDE  topic=$TOPIC"
docker ps --format '{{.Names}}' | grep -q "^${CONTAINER}$" \
  || { echo "ERROR: '$CONTAINER' not running -> ./up.sh $SIDE"; exit 1; }

# Run grip + open in ONE container session (warm ROS discovery — otherwise the 2nd
# `ros2 topic pub` from a fresh exec can be dropped). Messages passed via -e to dodge quoting.
# `-w 1` makes pub wait for the hand driver's subscription before sending, so it always lands.
docker exec -e TOPIC="$TOPIC" -e TYPE="$TYPE" -e STATE="$STATE" -e GRIP="$GRIP" -e OPEN="$OPEN" \
  "$CONTAINER" bash -lc '
  source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=0
  pub()  { ros2 topic pub --once -w 1 "$TOPIC" "$TYPE" "$1" >/dev/null 2>&1; }
  read_st() { timeout 3 ros2 topic echo --once "$STATE" 2>/dev/null | sed -n "/position:/,/]/p" | grep -E "^- " | head -6 | tr "\n" " "; }
  echo ">> 잡기 (GRIP)"; pub "$GRIP"; sleep 2; echo "   state: $(read_st)"
  echo ">> 펴기 (OPEN)"; pub "$OPEN"; sleep 2; echo "   state: $(read_st)"
'
echo "[test_hand] done"
