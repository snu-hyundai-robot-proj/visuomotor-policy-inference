# j6 Offset / Calibration Audit

## Scope
This is an offline repository/config audit only. No ROS publish, serial write, Docker restart, replay, build, or driver execution was performed.

## Source Hashes
- `inspire_driver.py`: `9ae4af116a9dd4c8e0c3811d8a577018851d933e5aa6fdb5240c0f86563c65ed`
- `inspire_comm.py`: `66a96b69c8113fee2ff6fea24976afe136364c6e45be36b3aec2e18b73548266`
- `inspire_bridge.py`: `1b62a98e414c026e5761db9861f9a3fe818008eee645e4ab2c50d37d097055ea`

## Active hand j6 command/feedback path findings
- No active software/config `offset`, `zero`, `angle_offset`, or `joint_offset` parameter was found in the hand j6 driver path.
- The active j6 target mapping is `thumb_rot = -data[5] * 950 + 1900` in 0.1 degree units, followed by an upper pre-clamp at 1800 and generic min/max clamp.
- The active j6 target limits are 600..1800 raw 0.1 degree units, i.e. 60..180 degrees.
- `/inspire/joint_states` `j6` is parsed from `angleAct`; `tj6` is driver-side target echo from `target_joint`, not a hardware ACK.
- `mode`, `forceClb`, `clearErr`, `currAct`, `errCode`, and `statusCode` registers are defined in the protocol dictionary, but the current polling path does not read/write them.

## Active inventory subset

|file|line|parameter / variable name|default value|j6에 적용되는지|target에 영향 / feedback에 영향 / 둘 다|startup에만 적용 / runtime에 적용|현재 실행 경로에서 실제 사용되는지|
|---|---|---|---|---|---|---|---|
|system_Teleop/src/Inspire_/inspire_driver/inspire_driver/inspire_dr...|32|INSPIRE_FINGER_MIN_DEGREE|[880, 880, 880, 880, 1100, 600]|yes|target|runtime|yes|
|system_Teleop/src/Inspire_/inspire_driver/inspire_driver/inspire_dr...|33|INSPIRE_FINGER_MAX_DEGREE|[1740, 1740, 1740, 1740, 1350, 1800]|yes|target|runtime|yes|
|system_Teleop/src/Inspire_/inspire_driver/inspire_driver/inspire_dr...|107|if current_data[i] >= INSPIRE_FINGER_MAX_DEGREE[i]|INSPIRE_FINGER_MAX_DEGREE[i]:|yes|target|runtime|yes|
|system_Teleop/src/Inspire_/inspire_driver/inspire_driver/inspire_dr...|108|literal/comment|INSPIRE_FINGER_MAX_DEGREE[i]|yes|target|runtime|yes|
|system_Teleop/src/Inspire_/inspire_driver/inspire_driver/inspire_dr...|110|if current_data[i] <= INSPIRE_FINGER_MIN_DEGREE[i]|INSPIRE_FINGER_MIN_DEGREE[i]:|yes|target|runtime|yes|
|system_Teleop/src/Inspire_/inspire_driver/inspire_driver/inspire_dr...|111|literal/comment|INSPIRE_FINGER_MIN_DEGREE[i]|yes|target|runtime|yes|
|system_Teleop/src/Inspire_/inspire_driver/inspire_driver/inspire_dr...|122|literal/comment||yes|target|runtime|yes|
|system_Teleop/src/Inspire_/inspire_driver/inspire_driver/inspire_dr...|128|thumb_rot|-cur_6d_vec[5] * 950 + 1900|yes|target|runtime|yes|
|system_Teleop/src/Inspire_/inspire_driver/inspire_driver/inspire_dr...|130|if thumb_rot > 1800||yes|target|runtime|yes|
|system_Teleop/src/Inspire_/inspire_driver/inspire_driver/inspire_dr...|131|thumb_rot|1800|yes|target|runtime|yes|
|system_Teleop/src/Inspire_/inspire_driver/inspire_driver/inspire_dr...|132|literal/comment||yes|target|runtime|yes|
|system_Teleop/src/Inspire_/inspire_driver/inspire_driver/communicat...|18|'angleSet'|1040, # Use this to set finger position, units of 0.1 degrees, -1 f...|yes|target_protocol|runtime|yes|
|system_Teleop/src/Inspire_/inspire_driver/inspire_driver/communicat...|21|'angleAct'|1064, # Use this to get finger position : angle Actual, units of 0....|yes|feedback_protocol|runtime|yes|
|system_Teleop/src/Inspire_/inspire_driver/inspire_driver/communicat...|36|SRBL_INSPIRE_FINGER_LOWER_LIMIT|[900, 900, 900, 900, 1100, 600] # Lower limit of the finger joint p...|yes|target|runtime|yes|
|system_Teleop/src/Inspire_/inspire_driver/inspire_driver/communicat...|38|SRBL_INSPIRE_FINGER_UPPER_LIMIT|[1740, 1740, 1740, 1740, 1350, 1800] # Upper limit of the finger jo...|yes|target|runtime|yes|
|system_Teleop/src/Inspire_/inspire_driver/inspire_driver/communicat...|191|literal/comment||yes|feedback_protocol|runtime|yes|
|system_Teleop/src/Inspire_/inspire_driver/inspire_driver/communicat...|198|literal/comment||yes|target_protocol|runtime|yes|
|system_Teleop/src/Inspire_/inspire_driver/inspire_driver/communicat...|227|literal/comment|int(min(SRBL_INSPIRE_FINGER_UPPER_LIMIT[i], max(SRBL_INSPIRE_FINGER...|yes|target|runtime|yes|

## Teleop-only dynamic normalization candidates
These can change teleop `data[5]` range, but they are not active during the direct j6 test when the bridge is stopped.

|file|line|parameter / variable name|default value|j6에 적용되는지|target에 영향 / feedback에 영향 / 둘 다|startup에만 적용 / runtime에 적용|현재 실행 경로에서 실제 사용되는지|
|---|---|---|---|---|---|---|---|
|system_Teleop/src/Inspire_/inspire_driver/inspire_driver/inspire_br...|13|RH56_ORDER|("pinky", "ring", "middle", "index", "thumb_bend", "thumb_rot")|yes_teleop_input|target_input|runtime_when_bridge_running|not_active_in_direct_test|
|system_Teleop/src/Inspire_/inspire_driver/inspire_driver/inspire_br...|17|_HIGH_IS_OPEN|frozenset({"thumb_rot"})|yes_teleop_input|target_input|runtime_when_bridge_running|not_active_in_direct_test|
|system_Teleop/src/Inspire_/inspire_driver/inspire_driver/inspire_br...|54|def _thumb_rot_raw(d|Dict[str, float]) -> float:|yes_teleop_input|target_input|runtime_when_bridge_running|not_active_in_direct_test|
|system_Teleop/src/Inspire_/inspire_driver/inspire_driver/inspire_br...|65|"thumb_rot"|_thumb_rot_raw(d),|yes_teleop_input|target_input|runtime_when_bridge_running|not_active_in_direct_test|
|system_Teleop/src/Inspire_/inspire_driver/inspire_driver/inspire_br...|99|pct|open, 1st pct = close|yes_teleop_input|target_input|runtime_when_bridge_running|not_active_in_direct_test|

## Not active or not hand-runtime candidates

|file|line|parameter / variable name|default value|j6에 적용되는지|target에 영향 / feedback에 영향 / 둘 다|startup에만 적용 / runtime에 적용|현재 실행 경로에서 실제 사용되는지|
|---|---|---|---|---|---|---|---|
|system_Teleop/src/Delto_/dg5f_driver/urdf/dg5f_right_ros2_control.x...|28|xyz|"0 0 0" rpy="0 0 0" />|arm_j6_not_hand|arm_urdf_only|not_hand_runtime|not_hand_path|
|system_Teleop/src/Delto_/dg5f_driver/urdf/dg5f_right_ros2_control.x...|49|xyz|"0 0.0128 0.0045" rpy="0 0 -1.5708" />|arm_j6_not_hand|arm_urdf_only|not_hand_runtime|not_hand_path|
|system_Teleop/src/Delto_/dg5f_driver/urdf/dg5f_right_ros2_control.x...|56|xyz|"0.0045 0 0.0128" rpy="0 1.5708 0" />|arm_j6_not_hand|arm_urdf_only|not_hand_runtime|not_hand_path|
|system_Teleop/src/Delto_/dg5f_driver/urdf/dg5f_right_ros2_control.x...|63|xyz|"0.0045 0 0.0128" rpy="0 1.5708 0" />|arm_j6_not_hand|arm_urdf_only|not_hand_runtime|not_hand_path|
|system_Teleop/src/Delto_/dg5f_driver/urdf/dg5f_right_ros2_control.x...|70|xyz|"0.0045 0 0.0128" rpy="0 1.5708 0" />|arm_j6_not_hand|arm_urdf_only|not_hand_runtime|not_hand_path|
|system_Teleop/src/Delto_/dg5f_driver/urdf/dg5f_right_ros2_control.x...|77|xyz|"0.0045 0 0.0128" rpy="0 1.5708 0" />|arm_j6_not_hand|arm_urdf_only|not_hand_runtime|not_hand_path|
|system_Teleop/src/Delto_/dg5f_driver/urdf/dg5f_left_ros2_control.xacro|28|xyz|"0 0 0" rpy="0 0 0" />|arm_j6_not_hand|arm_urdf_only|not_hand_runtime|not_hand_path|
|system_Teleop/src/Delto_/dg5f_driver/urdf/dg5f_left_ros2_control.xacro|49|xyz|"0 -0.0128 0.0045" rpy="0 0 1.5708" />|arm_j6_not_hand|arm_urdf_only|not_hand_runtime|not_hand_path|
|system_Teleop/src/Delto_/dg5f_driver/urdf/dg5f_left_ros2_control.xacro|56|xyz|"0.0045 0 0.0128" rpy="0 1.5708 0" />|arm_j6_not_hand|arm_urdf_only|not_hand_runtime|not_hand_path|
|system_Teleop/src/Delto_/dg5f_driver/urdf/dg5f_left_ros2_control.xacro|63|xyz|"0.0045 0 0.0128" rpy="0 1.5708 0" />|arm_j6_not_hand|arm_urdf_only|not_hand_runtime|not_hand_path|
|system_Teleop/src/Delto_/dg5f_driver/urdf/dg5f_left_ros2_control.xacro|70|xyz|"0.0045 0 0.0128" rpy="0 1.5708 0" />|arm_j6_not_hand|arm_urdf_only|not_hand_runtime|not_hand_path|
|system_Teleop/src/Delto_/dg5f_driver/urdf/dg5f_left_ros2_control.xacro|77|xyz|"0.0045 0 0.0128" rpy="0 1.5708 0" />|arm_j6_not_hand|arm_urdf_only|not_hand_runtime|not_hand_path|
|system_Teleop/src/Delto_/dg5f_driver/urdf/dg5f_both_ros2_control.xacro|34|xyz|"0 0.2 0" rpy="0 0 0" />|arm_j6_not_hand|arm_urdf_only|not_hand_runtime|not_hand_path|
|system_Teleop/src/Delto_/dg5f_driver/urdf/dg5f_both_ros2_control.xacro|41|xyz|"0 -0.2 0" rpy="0 0 0" />|arm_j6_not_hand|arm_urdf_only|not_hand_runtime|not_hand_path|
|system_Teleop/src/Delto_/dg5f_driver/urdf/dg5f_both_ros2_control.xacro|74|xyz|"0 -0.0128 0.0045" rpy="0 0 1.5708" />|arm_j6_not_hand|arm_urdf_only|not_hand_runtime|not_hand_path|
|system_Teleop/src/Delto_/dg5f_driver/urdf/dg5f_both_ros2_control.xacro|81|xyz|"0.0045 0 0.0128" rpy="0 1.5708 0" />|arm_j6_not_hand|arm_urdf_only|not_hand_runtime|not_hand_path|
|system_Teleop/src/Delto_/dg5f_driver/urdf/dg5f_both_ros2_control.xacro|88|xyz|"0.0045 0 0.0128" rpy="0 1.5708 0" />|arm_j6_not_hand|arm_urdf_only|not_hand_runtime|not_hand_path|
|system_Teleop/src/Delto_/dg5f_driver/urdf/dg5f_both_ros2_control.xacro|95|xyz|"0.0045 0 0.0128" rpy="0 1.5708 0" />|arm_j6_not_hand|arm_urdf_only|not_hand_runtime|not_hand_path|
|system_Teleop/src/Delto_/dg5f_driver/urdf/dg5f_both_ros2_control.xacro|102|xyz|"0.0045 0 0.0128" rpy="0 1.5708 0" />|arm_j6_not_hand|arm_urdf_only|not_hand_runtime|not_hand_path|
|system_Teleop/src/Delto_/dg5f_driver/urdf/dg5f_both_ros2_control.xacro|110|xyz|"0 0.0128 0.0045" rpy="0 0 -1.5708" />|arm_j6_not_hand|arm_urdf_only|not_hand_runtime|not_hand_path|
|system_Teleop/src/Delto_/dg5f_driver/urdf/dg5f_both_ros2_control.xacro|117|xyz|"0.0045 0 0.0128" rpy="0 1.5708 0" />|arm_j6_not_hand|arm_urdf_only|not_hand_runtime|not_hand_path|
|system_Teleop/src/Delto_/dg5f_driver/urdf/dg5f_both_ros2_control.xacro|124|xyz|"0.0045 0 0.0128" rpy="0 1.5708 0" />|arm_j6_not_hand|arm_urdf_only|not_hand_runtime|not_hand_path|
|system_Teleop/src/Delto_/dg5f_driver/urdf/dg5f_both_ros2_control.xacro|131|xyz|"0.0045 0 0.0128" rpy="0 1.5708 0" />|arm_j6_not_hand|arm_urdf_only|not_hand_runtime|not_hand_path|
|system_Teleop/src/Delto_/dg5f_driver/urdf/dg5f_both_ros2_control.xacro|138|xyz|"0.0045 0 0.0128" rpy="0 1.5708 0" />|arm_j6_not_hand|arm_urdf_only|not_hand_runtime|not_hand_path|
|system_Teleop/src/Delto_/dg_description/urdf/dg5f_right.xacro|17|xyz|"0 0 0" rpy="0 0 0" />|arm_j6_not_hand|arm_urdf_only|not_hand_runtime|not_hand_path|
|system_Teleop/src/Delto_/dg_description/urdf/dg5f_right.xacro|24|xyz|"-6.4721E-09 -2.1674E-05 0.0049997" rpy="0.0 0.0 0.0" />|arm_j6_not_hand|arm_urdf_only|not_hand_runtime|not_hand_path|
|system_Teleop/src/Delto_/dg_description/urdf/dg5f_right.xacro|42|xyz|"-0.00058969 0.0005138 0.037363" rpy="0.0 0.0 0.0" />|arm_j6_not_hand|arm_urdf_only|not_hand_runtime|not_hand_path|
|system_Teleop/src/Delto_/dg_description/urdf/dg5f_right.xacro|47|xyz|"0 0 0" rpy="0 0 0" />|arm_j6_not_hand|arm_urdf_only|not_hand_runtime|not_hand_path|
|system_Teleop/src/Delto_/dg_description/urdf/dg5f_right.xacro|53|xyz|"0 0 0" rpy="0 0 0" />|arm_j6_not_hand|arm_urdf_only|not_hand_runtime|not_hand_path|
|system_Teleop/src/Delto_/dg_description/urdf/dg5f_right.xacro|62|rpy|"0 0 0" xyz="0 0 0.004"/>|arm_j6_not_hand|arm_urdf_only|not_hand_runtime|not_hand_path|
|system_Teleop/src/Delto_/dg_description/urdf/dg5f_right.xacro|67|xyz|"-0.0089577 0.00095577 0.041325" rpy="0.0 0.0 0.0" />|arm_j6_not_hand|arm_urdf_only|not_hand_runtime|not_hand_path|
|system_Teleop/src/Delto_/dg_description/urdf/dg5f_right.xacro|72|xyz|"0 0 0" rpy="0 0 0" />|arm_j6_not_hand|arm_urdf_only|not_hand_runtime|not_hand_path|
|system_Teleop/src/Delto_/dg_description/urdf/dg5f_right.xacro|78|xyz|"0 0 0" rpy="0 0 0" />|arm_j6_not_hand|arm_urdf_only|not_hand_runtime|not_hand_path|
|system_Teleop/src/Delto_/dg_description/urdf/dg5f_right.xacro|87|rpy|"0 0 0" xyz="0 0 0.0698"/>|arm_j6_not_hand|arm_urdf_only|not_hand_runtime|not_hand_path|
|system_Teleop/src/Delto_/dg_description/urdf/dg5f_right.xacro|92|xyz|"0.021473 3.1163E-06 0.0028146" rpy="0.0 0.0 0.0" />|arm_j6_not_hand|arm_urdf_only|not_hand_runtime|not_hand_path|
|system_Teleop/src/Delto_/dg_description/urdf/dg5f_right.xacro|97|xyz|"0 0 0" rpy="0 0 0" />|arm_j6_not_hand|arm_urdf_only|not_hand_runtime|not_hand_path|
|system_Teleop/src/Delto_/dg_description/urdf/dg5f_right.xacro|103|xyz|"0 0 0" rpy="0 0 0" />|arm_j6_not_hand|arm_urdf_only|not_hand_runtime|not_hand_path|
|system_Teleop/src/Delto_/dg_description/urdf/dg5f_right.xacro|110|xyz|"-0.0162 0.019 0.0128" rpy="0.0 0.0 0.0" />|arm_j6_not_hand|arm_urdf_only|not_hand_runtime|not_hand_path|
|system_Teleop/src/Delto_/dg_description/urdf/dg5f_right.xacro|114|lower|"-0.3839724354387525" upper="0.8901179185171081" effort="7.5" veloc...|arm_j6_not_hand|arm_urdf_only|not_hand_runtime|not_hand_path|
|system_Teleop/src/Delto_/dg_description/urdf/dg5f_right.xacro|119|xyz|"0.00055633 0.0155 0.00055633" rpy="0.0 0.0 0.0" />|arm_j6_not_hand|arm_urdf_only|not_hand_runtime|not_hand_path|
|system_Teleop/src/Delto_/dg_description/urdf/dg5f_right.xacro|124|xyz|"0 0 0" rpy="0 0 0" />|arm_j6_not_hand|arm_urdf_only|not_hand_runtime|not_hand_path|
|system_Teleop/src/Delto_/dg_description/urdf/dg5f_right.xacro|130|xyz|"0 0 0" rpy="0 0 0" />|arm_j6_not_hand|arm_urdf_only|not_hand_runtime|not_hand_path|
|system_Teleop/src/Delto_/dg_description/urdf/dg5f_right.xacro|137|xyz|"0.04195 0.0 0.0" rpy="0.0 0.0 0.0" />|arm_j6_not_hand|arm_urdf_only|not_hand_runtime|not_hand_path|
|system_Teleop/src/Delto_/dg_description/urdf/dg5f_right.xacro|141|lower|"-3.141592653589793" upper="0.0" effort="7.5" velocity="3.141592653...|arm_j6_not_hand|arm_urdf_only|not_hand_runtime|not_hand_path|
|system_Teleop/src/Delto_/dg_description/urdf/dg5f_right.xacro|146|xyz|"2.3008E-05 0.014842 4.7446E-06" rpy="0.0 0.0 0.0" />|arm_j6_not_hand|arm_urdf_only|not_hand_runtime|not_hand_path|
|system_Teleop/src/Delto_/dg_description/urdf/dg5f_right.xacro|151|xyz|"0 0 0" rpy="0 0 0" />|arm_j6_not_hand|arm_urdf_only|not_hand_runtime|not_hand_path|
|system_Teleop/src/Delto_/dg_description/urdf/dg5f_right.xacro|157|xyz|"0 0 0" rpy="0 0 0" />|arm_j6_not_hand|arm_urdf_only|not_hand_runtime|not_hand_path|
|system_Teleop/src/Delto_/dg_description/urdf/dg5f_right.xacro|164|xyz|"0.0 0.031 0.0" rpy="0.0 0.0 0.0" />|arm_j6_not_hand|arm_urdf_only|not_hand_runtime|not_hand_path|
|system_Teleop/src/Delto_/dg_description/urdf/dg5f_right.xacro|168|lower|"-1.5707963267948966" upper="1.5707963267948966" effort="7.5" veloc...|arm_j6_not_hand|arm_urdf_only|not_hand_runtime|not_hand_path|
|system_Teleop/src/Delto_/dg_description/urdf/dg5f_right.xacro|173|xyz|"-0.00020482 0.018795 -0.00036539" rpy="0.0 0.0 0.0" />|arm_j6_not_hand|arm_urdf_only|not_hand_runtime|not_hand_path|
