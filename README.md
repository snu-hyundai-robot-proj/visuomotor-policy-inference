# Visuomotor Policy Inference

A small, production-style **inference server** for serving the Hyundai Uiwang
**FlowMatch Diffusion Policy** (LeRobot) over HTTP. Give it two camera frames and a
robot state vector; it returns the next action. Ships with a Dockerfile and a
`docker compose` one-shot setup.

- Model: [`Ngseo/hyundai-uiwang-left-flowmatch`](https://huggingface.co/Ngseo/hyundai-uiwang-left-flowmatch) (loaded from the Hub at startup)
- Framework: FastAPI + Uvicorn
- Policy: LeRobot Diffusion Policy with the `FlowMatch` scheduler (`num_inference_steps=1`)

## Performance

Benchmarked on an **NVIDIA GeForce RTX 3060 (12 GB)** (torch 2.10.0+cu128 / CUDA 12.8):
**model inference runs at ~40 Hz** (one diffusion forward ≈ 24 ms), with ~1.4 GB VRAM.
Reproduce with `python scripts/benchmark.py --device cuda`.

> **Important:** the model uses the **FlowMatch** scheduler, a custom addition in the
> [`snu-hyundai-robot-proj/lerobot`](https://github.com/snu-hyundai-robot-proj/lerobot) fork.
> `requirements.txt` installs `lerobot` from that fork — **upstream/PyPI lerobot will not load this model.**

## Quick start — Docker Compose (one shot)

```bash
docker compose up --build
```

That builds the image, starts the server on **http://localhost:8000**, and downloads
the model (~1.1 GB) into a persistent `hf-cache` volume on first run. Then:

```bash
# health / metadata
curl localhost:8000/health
curl localhost:8000/info

# end-to-end smoke test with synthetic frames
pip install requests pillow numpy
python examples/client_example.py --url http://localhost:8000
```

GPU serving: install the NVIDIA Container Toolkit, set `VPI_DEVICE=cuda` and uncomment
the `deploy:` GPU block in `docker-compose.yml`, then `docker compose up --build`.

## Run without Docker (dev)

```bash
pip install -r requirements.txt           # installs lerobot from the fork
uvicorn app.server:app --host 0.0.0.0 --port 8000
```

## API

| Method | Path | Body | Returns |
|---|---|---|---|
| GET | `/health` | — | `{status, model_loaded}` |
| GET | `/info` | — | model id, device, cameras, dims, scheduler |
| POST | `/reset` | — | clears the action queue (start a new episode) |
| POST | `/predict` | `PredictRequest` | `{action: float[26]}` |

### `POST /predict`

```jsonc
{
  "front_rgb": "<base64 PNG/JPEG>",   // scene / zivid camera (RGB, any resolution)
  "wrist_rgb": "<base64 PNG/JPEG>",   // wrist camera (RGB)
  "state":     [/* 26 floats */]       // arm joints (6) + hand joints (20)
}
```

Response:

```json
{ "action": [/* 26 floats: target arm (6) + hand (20) joints @ 30 Hz */] }
```

Example with `curl` (Python one-liner to build the payload):

```bash
python - <<'PY' > /tmp/req.json
import base64, io, json, numpy as np
from PIL import Image
def enc(a): 
    b=io.BytesIO(); Image.fromarray(a).save(b,"PNG"); return base64.b64encode(b.getvalue()).decode()
f=np.random.randint(0,256,(480,640,3),np.uint8); w=f.copy()
json.dump({"front_rgb":enc(f),"wrist_rgb":enc(w),"state":[0.0]*26}, open("/tmp/req.json","w"))
PY
curl -s -X POST localhost:8000/predict -H 'Content-Type: application/json' -d @/tmp/req.json
```

## Control-loop usage

The policy keeps an internal observation/action queue (`n_obs_steps=2`,
`n_action_steps=8`). Call `/reset` once at the start of an episode, then `/predict`
at your control rate (the model targets **30 Hz**):

```python
import requests
requests.post("http://localhost:8000/reset")
while running:
    action = requests.post("http://localhost:8000/predict", json={
        "front_rgb": enc(front_camera_rgb),   # uint8 (H,W,3)
        "wrist_rgb": enc(wrist_camera_rgb),
        "state": robot_state.tolist(),         # 26 floats
    }).json()["action"]
    robot.send_action(action)
```

Camera input resolution need not match training (the policy resizes/crops internally),
but the two views must be the correct cameras (front vs wrist).

## Evaluate & run on the robot

Three things to run, in increasing order of "needs hardware". Start the server first
(`docker compose up`, or `uvicorn ...`).

### 1. Offline policy check — *no robot* (verify the model learned)
Replays a recorded sample episode (`examples/sample_episodes/<side>/`) through `/predict`
and reports the **L1 error** vs the recorded action (with a `action == state` baseline):

```bash
python scripts/eval_l1.py --side left --url http://localhost:8000 --out /tmp/l1_report.json
# e.g. MODEL overall L1 ~0.007 rad  vs  baseline ~0.17  -> the model tracks the recording well
```

### 2. Live closed-loop dry-run — *no robot* (verify the loop + rate)
Streams the episode **in real time (30 Hz)**, runs the policy each tick, and prints the
exact command that *would* be sent to the robot (arm in deg, hand in rad). It does **not**
move anything:

```bash
python examples/run_closed_loop.py --side left --fps 30 --record /tmp/cmds.jsonl
# done: ... -> ~30 Hz achieved | /predict latency mean ~16ms
```

### 3. Real robot closed-loop — *needs the robot* (the actual deployment)
The continuous "live cameras → policy → robot" code is the **`vpi_robot_client`** ROS2
node; the episode lifecycle (home → run → stop → re-home) is the **`episode_manager`** node;
the **web console** drives it. This is what must be validated on the real robot/sim.

```bash
# build the two ROS2 packages into your workspace (one-time)
ln -s $PWD/ros2_robot_client    <ws>/src/vpi_robot_client
ln -s $PWD/ros2_episode_manager <ws>/src/episode_manager
cd <ws> && colcon build && source install/setup.bash

# bring-up (each in its own terminal): drivers + cameras run on the host
VPI_DEVICE=cuda uvicorn app.server:app --host 0.0.0.0 --port 8000   # policy server (GPU)
ros2 launch hdr_stream ...            # HDR35 arm driver  (simulation:=true to start safe)
ros2 launch dg5f_driver ...           # DG5F hand + pospid controller
ros2 run teleop_vision vision_node_left   # publishes the two camera topics

ros2 launch vpi_robot_client policy_control.launch.py enable_output:=false   # infer only, no motion
ros2 launch episode_manager  episode.launch.py                               # state machine + homing

# web console
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
cd frontend && python -m http.server 8080      # open http://localhost:8080 -> HOME / START / STOP
```

**Staged bring-up (do not skip):** `enable_output:=false` (watch actions, no motion) →
simulation → real robot with small `max_joint_delta`/`max_gripper_delta` → normal.
Set the init pose (`arm_home`/`hand_home`) first. Details:
[`EXECUTION_PLAN.md`](./EXECUTION_PLAN.md), [`EPISODE_SYSTEM.md`](./EPISODE_SYSTEM.md),
[`ros2_robot_client/README.md`](./ros2_robot_client/README.md),
[`ros2_episode_manager/README.md`](./ros2_episode_manager/README.md). Overview:
[`PROJECT_OVERVIEW.md`](./PROJECT_OVERVIEW.md).

## Live inference via the teleop container — verified LEFT runbook

> This is the path that plugs into the **friend's dockerized teleop** (`ros2_teleop_system`):
> the Zivid+RealSense vision node and the DG-5F hand drivers already run **inside that
> container** (host network, `ROS_DOMAIN_ID=0`, `/dev/shm` shared). We just add the policy
> server and a small live-inference script. Verified working on **LEFT, 2026-06** —
> `examples/live_infer.py` produces stable, in-distribution actions (~35–40 ms/call).

**State vector** `[26] = arm6 (rad) + hand20 (rad)`. The hand-20 must be in the **canonical
order `lj_dg_1_1, 1_2, 1_3, 1_4, 2_1, … 5_4`** (`idx = 4*(finger-1)+(joint-1)`) — the order
the dataset recorder (`system_left.cpp gripperCallback`) used. **`/dg5f_left/joint_states`
publishes joints in a scrambled order**, so `live_infer.py` re-indexes by name. Feeding raw
`msg.position` (scrambled) produces erratic, out-of-distribution actions.

### 0. one-time: network for the arm (only if driving the powered arm)
```bash
sudo ip addr add 192.168.4.55/24 dev enp4s0 2>/dev/null   # enp4s0 IP is non-persistent
```

### 1. policy server (host shell)
```bash
cd ~/visuomotor-policy-inference
docker compose -f docker-compose.full.yml up -d inference
curl -s http://localhost:8000/info        # {"model_id":"…-left-flowmatch","state_dim":26,…}
```

### 2. vision node = cameras (inside the teleop container) — ~25 s to connect, one side only
```bash
docker exec -d ros2_teleop_system bash -lc '
  source /opt/ros/humble/setup.bash; source /workspace/install/setup.bash;
  export ROS_DOMAIN_ID=0;
  exec ros2 run teleop_vision system_vision_left > /tmp/vision_left.log 2>&1'

# confirm both cameras publish (~14 Hz each)
docker exec ros2_teleop_system bash -lc 'source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=0;
  ros2 topic hz /system_left/zivid_rgb'      # also /system_left/d405_rgb
```

### 3. (optional) send the hand to its mean init pose first
```bash
docker cp examples/home_to_init.py ros2_teleop_system:/tmp/home_to_init.py
docker exec ros2_teleop_system bash -lc 'source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=0;
  cd /tmp; python3 home_to_init.py --side left --hand-only --engage --hand-speed 0.5 --settle 4'
# arm+hand together: drop --hand-only (needs the arm powered + step 0)
```

### 4. live cameras → model → action
```bash
docker cp examples/live_infer.py ros2_teleop_system:/tmp/live_infer.py
docker exec ros2_teleop_system bash -lc 'source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=0;
  cd /tmp; python3 live_infer.py --side left --n 10 --no-arm --no-hand'
```
First call ~4 s (warmup), then ~35–40 ms. `finite=True` + actions near the current pose = healthy.

**`live_infer.py` flags** — pick per what hardware is powered:

| flag | when | effect on the `[26]` state |
|---|---|---|
| *(none)* | arm **and** hand powered & live | arm6 read over OpenStream TCP, hand20 read from `/dg5f_left/joint_states` (reordered) |
| `--no-arm` | arm **off** | arm6 = fixed mean init (`ARM_INIT_RAD`) |
| `--no-hand` | hand off, or static at init | hand20 = fixed mean init (`HAND_INIT`, canonical order) |

> With a **static scene** the model correctly outputs "stay near init". To see meaningful
> motion you need a **real task object** in front of the cameras (training distribution).

### 5. closed loop — live cameras → model → **robot moving** (the real deployment)
`examples/run_robot_loop.py` is the continuous loop: each tick reads the live cameras + state,
calls `/predict`, and **drives the action onto the arm + hand**, repeating. `live_infer.py`
only prints; `drive_*_replay.py` feed a recorded episode; this one closes the loop on the live
scene.

```bash
docker cp examples/run_robot_loop.py ros2_teleop_system:/tmp/run_robot_loop.py
# DRY-RUN first (predicts, prints the target deltas it WOULD command — no motion):
docker exec ros2_teleop_system bash -lc 'source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=0;
  cd /tmp; python3 run_robot_loop.py --side left --hand-only'

# engage — drive the HAND from live model output (arm off):
docker exec ros2_teleop_system bash -lc 'source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=0;
  cd /tmp; python3 run_robot_loop.py --side left --hand-only --engage --secs 20'

# engage — ARM + HAND (arm powered + step 0 network); drop --hand-only:
#   python3 run_robot_loop.py --side left --engage --secs 20
```

Safety built in: **DRY-RUN default**, homes to mean init first, **per-tick velocity clamp**
(`--arm-speed` deg/s, `--hand-speed` rad/s — bounds how far it moves toward each target so a
wild action can't jump), `--secs` cap, `api.stop` on exit. Verified dry-run on LEFT: the model
asks for only **±3° arm corrections** from init on a static scene — engage on a static scene
moves little (correct); put a **task object** in view for real behaviour.

### Findings & gotchas (2026-06)
- **State joint order** — fixed in `live_infer.py` (reindex by name). The drive/replay
  scripts read the hand the same raw way; apply the same fix before trusting their state.
- **`/dg5f_left/joint_states` QoS is `RELIABLE` + `TRANSIENT_LOCAL` (latched).** A default
  `VOLATILE` rclpy subscription can miss the latched value when the hand is static → use the
  matching QoS (done in `live_infer.py`/`home_to_init.py`). Symptom: `hand=False` forever.
- **DG-5F pospid gains are uniform `P=1.5, I=0`** (no tuned config). Settable live via
  `ros2 param set /dg5f_left/lj_dg_pospid gains.<joint>.{p,i,i_clamp_max,i_clamp_min}`, but
  **live changes are not persisted — a driver restart reverts to `P=1.5, I=0`.**
- **Do not crank gains to force the hand to init.** A few distal/curled joints
  (`lj_dg_1_3/2_3/3_3`, finger-1) don't fully open at `P=1.5`; raising `P`/adding `I` makes
  them tremble **and overheat** (stall current into joints that can't reach). `home_to_init`
  lands ~11/17 model-controlled joints within 0.1 rad at safe `P=1.5` — accept that residual.
  `lj_dg_2_1/3_1/4_1` are passive spread joints the model ignores (`action_indices` drops them).
- **One vision side at a time** (Zivid is exclusive). If `system_vision_left` was killed,
  cameras stop → `live_infer` waits on `front/wrist`; restart it (step 2).
- Kill stale scripts with the self-safe pattern: `pkill -9 -f '[l]ive_infer'`.

## Sides (left / right) & the web console

Everything is keyed by one knob, **`SIDE`** — it selects the camera pair, topics, model,
and arm together. Full table + commands in **[`SIDES.md`](./SIDES.md)**. In short:

| | LEFT | RIGHT |
|---|---|---|
| Zivid / RealSense | `23352865` / `409122273797` | `2051707B` / `409122273122` |
| topics | `/system_left/{zivid_rgb,d405_rgb}` | `/system_right/{zivid_rgb,d405_rgb}` |
| model | `…-left-flowmatch` | `…-right-flowmatch` |
| arm | 192.168.4.152 | 192.168.4.151 |

```bash
# vision node (one side at a time — see note)
ros2 run teleop_vision system_vision_left      # or system_vision_right

# swap model + camera-test side together via the per-side env file
docker compose --env-file env.left -f docker-compose.full.yml up -d --force-recreate inference
SIDE=left python examples/camera_infer_test.py           # standalone live test
python examples/drive_arm_replay.py --side left --steps 0 --engage   # drive that arm
```

### Web console with live cameras
The console (`frontend/`, served on :8080) now shows the **two live cameras** (front Zivid +
wrist RealSense) and has a **LEFT / RIGHT toggle**. Camera frames come over MJPEG from the
`web_video` service (`docker-compose.full.yml`, port **8090**) — raw Image over rosbridge is
too heavy. Open `http://localhost:8080`, pick a side, and the two views update.

> **⚠️ Only one side's cameras can be live at a time.** The Zivid SDK is exclusive — running
> `system_vision_left` and `system_vision_right` together fails (the second can't find its
> camera and breaks the first). So the toggle switches the *view*; to actually see the other
> side you must stop the running vision node and start the other one (and match the model/arm
> `SIDE`). Both RealSense cameras can be enumerated at once; only Zivid is exclusive.

## Configuration (env vars)

| var | default | description |
|---|---|---|
| `VPI_MODEL_ID` | `Ngseo/hyundai-uiwang-left-flowmatch` | any compatible LeRobot diffusion policy on the Hub |
| `VPI_DEVICE` | `cpu` | `cpu`, `cuda`, or `mps` |
| `HF_HOME` | `/cache/huggingface` | Hub cache location (mounted as a volume in compose) |

## Notes / limitations

- Single-robot serving: one shared action queue, guarded by a lock. For multiple
  independent robots, run one container per robot (or extend with per-session runners).
- The model is **public**, so no HF token is required. For a private model, pass
  `HF_TOKEN` as an env var.
- First startup is slow (model download + load); the compose healthcheck allows a
  180 s `start_period`.

## Layout

```
app/                       # FastAPI inference server (server.py, policy_runner.py, schemas.py)
scripts/
  benchmark.py             # latency + bandwidth benchmark  (-> BENCHMARK.md)
  eval_l1.py               # offline L1 eval vs a recorded episode
  smoke_test.sh            # health + info + predict
examples/
  client_example.py        # reference client (synthetic frames)
  replay_episode.py        # replay a sample episode through /predict
  run_closed_loop.py       # real-time closed-loop dry-run (no robot)
  camera_infer_test.py     # one-shot live-camera -> /predict sanity test
  live_infer.py            # live ROS cameras -> /predict -> action, PRINT only (--no-arm/--no-hand)
  run_robot_loop.py        # CLOSED LOOP: live cameras -> /predict -> drive robot (--hand-only/--engage)
  home_to_init.py          # send arm+hand (or --hand-only) to the mean init pose
  drive_arm_replay.py      # drive the real arm from model/recorded action
  drive_hand_replay.py     # drive the real hand from model/recorded action
  drive_arm_hand_replay.py # drive arm+hand together (home-to-init then replay)
  sample_episodes/         # recorded episodes (frames + episode.json)
ros2_robot_client/         # vpi_robot_client: live cameras+state -> /predict -> robot (ROS2)
ros2_episode_manager/      # episode_manager: home/run/stop state machine (ROS2)
frontend/                  # web console (HOME/START/STOP) over rosbridge
Dockerfile  docker-compose.yml  docker-compose.full.yml  requirements.txt
# docs: PROJECT_OVERVIEW · EPISODE_SYSTEM · EXECUTION_PLAN · INTEGRATION_HDR35 · BENCHMARK
```
