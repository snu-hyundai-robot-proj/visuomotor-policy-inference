# Left / Right — one knob to swap

Everything is keyed by **`SIDE` (left|right)**. Pick a side and the camera pair, topics,
model, and arm all follow.

## Mapping

| | LEFT | RIGHT |
|---|---|---|
| Zivid (front) | `23352865` @ 192.168.4.200 | `2051707B` @ 192.168.4.201 |
| RealSense (wrist) | `409122273797` | `409122273122` |
| Camera topics | `/system_left/{zivid_rgb,d405_rgb}` | `/system_right/{zivid_rgb,d405_rgb}` |
| Vision node | `system_vision_left` | `system_vision_right` |
| Policy model | `Ngseo/hyundai-uiwang-left-flowmatch` | `Ngseo/hyundai-uiwang-right-flowmatch` |
| HDR35 arm (OpenStream TCP) | 192.168.4.152:49000 | 192.168.4.151:49000 |
| DG5F gripper (Modbus TCP) | 169.254.186.73:502 | 169.254.186.72:502 |

## Swap each piece

**1) Cameras (vision node, in the teleop container)**
```bash
ros2 run teleop_vision system_vision_left     # or system_vision_right
# -> publishes /system_<side>/zivid_rgb (front) + /system_<side>/d405_rgb (wrist)
```

**2) Model + camera-test side (our docker stack)** — use the per-side env file:
```bash
# left
docker compose --env-file env.left  -f docker-compose.full.yml up -d --force-recreate inference
docker compose --env-file env.left  -f docker-compose.full.yml --profile test up camera_infer_test
# right
docker compose --env-file env.right -f docker-compose.full.yml up -d --force-recreate inference
docker compose --env-file env.right -f docker-compose.full.yml --profile test up camera_infer_test
```
(`env.left`/`env.right` set `SIDE` + `VPI_MODEL_ID` together.)

**3) Live camera -> inference test (standalone)**
```bash
SIDE=left  python examples/camera_infer_test.py     # subscribes /system_left/...
SIDE=right python examples/camera_infer_test.py
```

**4) Drive the arm from a recorded episode**
```bash
python examples/drive_arm_replay.py --side left  --steps 0          # DRY-RUN
python examples/drive_arm_replay.py --side left  --steps 0 --engage # move LEFT arm
python examples/drive_arm_replay.py --side right --steps 0 --engage # move RIGHT arm (.151)
```

> Keep the three sides aligned: the **vision node**, the **served model** (inference), and
> the **arm side** must all be the same `SIDE`, or the policy sees the wrong cameras / drives
> the wrong arm.

## One-click swap from the web console

Run the **side-switcher** on the host (needs `sudo docker`; vpi env has fastapi/uvicorn):
```bash
conda activate vpi
uvicorn app.side_switcher:app --host 0.0.0.0 --port 8070
```
Then the console's **LEFT / RIGHT toggle** auto-swaps the whole pipeline in one click:
- recreates `inference` with that side's model (via `env.<side>`),
- restarts the vision node to `system_vision_<side>` in the teleop container,
- re-points the camera views.

(The arm just needs `--side` when you run `drive_arm_replay.py` — nothing to restart.)
Endpoints: `GET /side`, `POST /side/{left|right}`, `GET /health`.

## Notes
- Zivid allows only one app to discover at a time, so a standalone `zivid` query returns 0
  while a vision node holds the subsystem — that's expected, not a missing camera.
- Gripper (DG5F) is on a `169.254.x` link-local net (Modbus/502) and is currently not routed
  on this host — arm-only until that link is configured.
