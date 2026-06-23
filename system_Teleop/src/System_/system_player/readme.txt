노드 실행 법
ros2 run system_player system_player_node --ros-args -p gripper_topic:=/dg5f_left/lj_dg_pospid/reference -p robot_topic:=/robot/joint_target_deg -p parquet_path:=/home/pin/Desktop/Tesollo/1.Project/1.hyundae/seoul_uiwang/data/converted/left/data/chunk-000/5 -p left_side:=True

ros2 run system_player system_player_node --ros-args -p gripper_topic:=/inspire/right/target -p robot_topic:=/robot/joint_target_deg -p parquet_path:=/home/pin/Desktop/Tesollo/1.Project/1.hyundae/seoul_uiwang/data/converted/right/data/chunk-000/5 -p left_side:=False

