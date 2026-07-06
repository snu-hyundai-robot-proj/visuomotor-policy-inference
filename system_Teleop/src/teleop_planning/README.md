# teleop_planning — perception → motion planning for HDR35 (standalone, no VLA)

A classical alternative to the VLA policy: take a detected 6D object pose (the hook), turn
it into an end-effector goal (configurable grasp offset + approach stand-off), and plan the
HDR35 there with MoveIt. Independent of the teleop/policy stack.

```
hook_pose (PoseStamped, base, mm)
   -> T_pf_grasp   = T_pf_hook · T_hook_from_grasp        (grasp offset params)
   -> T_pf_pregrasp = T_pf_grasp · standoff(-approach)     (approach along a grasp axis)
   -> MoveGroup action (OMPL/Pilz) -> trajectory -> [execute]
```

## Nodes
| node | role |
|------|------|
| `plan_to_hook` | subscribes hook pose, computes grasp+pregrasp EE poses, publishes them as PoseStamped+TF for RViz, and plans via `moveit_msgs/action/MoveGroup`. Services `~/plan` (plan only) and `~/execute` (gated by `allow_execute`). |
| `joint_state_relay` | HDR35 `/system_<side>/joint_states` (deg) → `/joint_states` (rad, j1..j6) so move_group has a current state. |
| `hdr_followjoint_bridge` | the execution bridge: serves `/joint_trajectory_controller/follow_joint_trajectory` (what MoveIt sends to) and streams points to the HDR35 over OpenStream (deg). `dry_run` (default true) logs instead of moving; rejects inter-point jumps > `max_step_deg`. |

## Run (planning only — does not touch the arm)
```bash
ros2 launch teleop_planning planning_bringup.launch.py side:=right
# feed a hook pose (run the hook_pose_estimator, or publish /hook_pose), watch
# /plan_to_hook/grasp_pose + /pregrasp_pose in RViz, tune the offset, then:
ros2 service call /plan_to_hook/plan std_srvs/srv/Trigger {}
```

Key params (`plan_to_hook`): `grasp_offset_xyz_mm`, `grasp_offset_rpy_deg`,
`approach_dist_mm` (default 100), `approach_axis` (default z), `pos_tol_m`, `ori_tol_rad`,
`planning_frame` (base_link), `ee_link` (tool0), `group` (hdr_manipulator),
`vel_scale`/`acc_scale`, `allow_execute` (default false). Hook Euler frame: the hook pose's
`base` frame is treated as the robot `base_link` (a static identity TF is published).

## Status / verified
Verified end-to-end in a MoveIt-enabled container (mock hardware):
- `~/plan` → SUCCESS plan; preview poses published; `~/execute` blocked while `allow_execute=false`.
- Full execution chain in **dry-run**: `~/execute` (allow_execute=true) → move_group plans+executes
  (120-pt trajectory, SUCCESS) → FollowJointTrajectory goal reaches `hdr_followjoint_bridge`,
  which logs the deg setpoints and reports completion. Only the real OpenStream send is gated.

## Real-arm execution (supervised)
The bridge is wired in. To actually move the HDR35 (arm clear, e-stop in reach):
```bash
ros2 launch teleop_planning planning_bringup.launch.py side:=right \
    use_mock_hardware:=false dry_run:=false allow_execute:=true
ros2 service call /plan_to_hook/execute std_srvs/srv/Trigger {}
```
Start with a small `approach_dist_mm`, low `vel_scale`/`acc_scale`. Never set `dry_run:=false`
unattended.

## Prereqs
MoveIt must be in the image (added to the Dockerfile). Deploy from the repo with
`scripts/deploy_planning_to_live.sh`. Fixed upstream: a copy-paste bug in
`hdr35_20_moveit_config/launch/move_group.launch.py` (`hdf7_9_moveit_config` →
`hdr35_20_moveit_config`).
