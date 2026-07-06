# Trace Integrity Check

- trace dir: `/tmp/replay_actual_trace_right_20260630_022959`
- phases: ['home', 'ramp_to_episode', 'replay', 'settle']
- replay ticks: 2196
- replay start offset from trace start: 6.740025 sec
- comparison plot x-axis: seconds from replay start; tick timing plot x-axis: seconds from trace start.
- actual execution file inspected: `/tmp/drive_arm_hand_replay.py` inside `ros2_teleop_system`
- replay loop: one `synced_step()` call, one `ArmClient.insert()` call, and one `publish_hand_trace()`/hand publish call per replay tick.
- home/ramp/settle: `sync_ramp()` ticks each send one arm insert and one hand publish; the post-home hold loop and settle loop each send one pair per tick. No duplicate send within a single tick was found.
- `/system_right/frame_index` was requested in rosbag, but the topic was not present/published in this run; bag contains hand topics only.
