# Quickstart — run model replay & closed-loop inference on the real robot

Minimal runbook. Full details: [`README.md`](./README.md).

> One side at a time (Zivid is exclusive). The arm (HDR35) is on the network; our scripts
> connect to it directly over OpenStream TCP — no separate arm UI/node needed.

## 1. Bring up the side's stack

```bash
cd ~/Tesollo/1.Project/1.hyundae/seoul_uiwang/system_Teleop

docker compose --profile right up -d     # RIGHT: vision + Inspire hand + right model
# docker compose --profile left  up -d   # LEFT:  vision + DG5F hand + left model

# switch sides: take both down first, then up the other
docker compose --profile left --profile right down
```

Each profile starts: `ros2_teleop_system` (base) · `teleop_vision_<side>` (cameras) ·
`teleop_hand_<side>` (hand driver) · `vpi_inference` (that side's model, port 8000).
Wait ~30 s for the cameras + model to load.

Check it's ready:
```bash
curl -s localhost:8000/info                                   # model loaded?
docker exec ros2_teleop_system bash -lc \
 'source /opt/ros/humble/setup.bash; ros2 topic hz /system_right/zivid_rgb'   # cameras?
```

## 2. Model replay — recorded episode images → model → robot

```bash
cd ~/visuomotor-policy-inference

DRY=1 ./run_episode_replay.sh right        # dry-run first (computes actions, NO motion)
./run_episode_replay.sh right 0            # engage: right side, all frames, arm + hand
./run_episode_replay.sh right 120          # only the first 120 frames
./run_episode_replay.sh left 0 --recorded  # replay the recorded action directly (skip the model)
```

## 3. Closed-loop inference — live cameras → model → robot

```bash
DRY=1 ./run_closed_loop.sh right           # dry-run (predicts, prints targets, NO motion)
./run_closed_loop.sh right 60              # engage: right arm + hand, 60 s
./run_closed_loop.sh left 60 --no-hand     # left arm only (DG5F hand off)
./run_closed_loop.sh right 60 --hand-only  # hand only (arm off)
```

Both scripts auto: check the stack is up → set the robot-net IP on `enp4s0` → copy the
script in → run with `--engage`. Built-in safety: home-to-init, time-synced arm+hand clamp,
`--secs` cap, stop-on-exit. `DRY=1` = no motion.

## Troubleshooting (one-liners)

| Symptom | Fix |
|---|---|
| Hand frozen / not moving (serial freeze) | `docker restart teleop_hand_<side>` |
| `address already in use :8000` | a stray inference container — run compose from the **teleop** dir, not this repo |
| cameras not publishing | vision node still loading RF-DETR-free init (~25 s); check `docker logs teleop_vision_<side>` |
| arm unreachable | `sudo ip addr add 192.168.4.55/24 dev enp4s0` (non-persistent) |
| left hand dead | DG5F needs power + ethernet on `192.168.4.x`; ros2_control must be in the image |
