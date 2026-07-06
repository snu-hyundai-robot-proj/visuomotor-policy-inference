# Zivid + D405 point-cloud fusion (classic / no learned model)

Both cameras are metric depth sensors, so "using them together" is a calibration +
transform-and-merge problem, not a generative model. Everything is computed in the
robot **base frame, in millimetres**.

```
T_d405_2base = T_flange2base (live, /system_<side>/pose_states)
             @ T_flange_from_d405 (constant, hand-eye calibration)
fused = transform(zivid, T_zivid2base) + ICP_refine(transform(d405, T_d405_2base))
```

## Modules
| file | role |
|------|------|
| `transforms.py` | pose/Euler ↔ 4×4 helpers (`T_a2b` maps frame a → b) |
| `rs_cloud.py`   | D405 depth+color → metric cloud (m → mm), with intrinsics |
| `fuse.py`       | bring both clouds to base, ICP-refine wrist onto Zivid, merge |
| `handeye_calibrate.py` | one-time `T_flange_from_d405` via ChArUco (node) |
| `fusion_node.py`| owns both cameras, `~/fuse` service → `.ply` + `PointCloud2` |

## 1. Hand-eye calibration (once per camera mount)
Tape a ChArUco board in view. The D405-only; Zivid not needed, so it can run while the
normal vision node is up.
```bash
ros2 run teleop_vision handeye_calibrate --ros-args \
  -p side:=right -p squares_x:=5 -p squares_y:=7 \
  -p square_len_mm:=30.0 -p marker_len_mm:=22.0 -p aruco_dict:=DICT_4X4_50
# jog arm to a varied pose, press <Enter> (×12+), then type 'solve'
```
It sweeps several Euler conventions for `pose_states` and saves the lowest-residual
`T_flange_from_d405_<side>.npy` into `camera/`. Note the printed `euler_order`.

## 2. Live fusion — embedded in the vision node (recommended)
`fuse()` is wired directly into `vision_node_{right,left}` (they already own both
cameras, so no Zivid-exclusivity dance). On startup the node loads
`camera/T_flange_from_d405_<side>.npy`, subscribes `pose_states`, and exposes a service:
```bash
ros2 service call /system_right/fuse_cloud std_srvs/srv/Trigger {}
```
Writes `Record/<side>/fused/<side>_NNN_fused.ply` (mm) and publishes
`/system_<side>/fused_cloud` (PointCloud2, metres, frame `base`) for RViz.
Set the Euler convention via env if calibration reported a non-default one:
`FUSION_EULER_ORDER=ZYX`.

### Standalone alternative (`fusion_node`)
If you'd rather run fusion outside the vision node, `fusion_node` owns both cameras —
but Zivid is **exclusive**, so stop `system_vision_<side>` first:
```bash
ros2 run teleop_vision fusion_node --ros-args -p side:=right -p euler_order:=xyz
ros2 service call /fusion_node/fuse std_srvs/srv/Trigger {}
```

## Notes
- `pose_states` packs Euler rx/ry/rz (deg) into `orientation.x/y/z` — not a quaternion.
- D405 min range ≈ 7 cm (`depth_range_m` default `(0.07, 1.0)`).
- To embed in the existing vision node (recommended for runtime, avoids the Zivid
  exclusivity dance): import `fuse`, `zivid_xyz_to_cloud`, `RealSenseD405` and call
  `fuse()` with the node's existing Zivid `xyz/rgba` and the loaded hand-eye matrix.
