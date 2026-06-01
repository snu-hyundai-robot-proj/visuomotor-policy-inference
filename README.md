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
app/
  server.py         # FastAPI app + endpoints
  policy_runner.py  # loads the policy from HF, runs inference (thread-safe)
  schemas.py        # request/response models
examples/
  client_example.py # reference client
scripts/
  smoke_test.sh     # health + info + predict against a running server
Dockerfile
docker-compose.yml
requirements.txt
```
