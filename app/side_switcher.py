"""Side switcher — one HTTP call swaps the whole pipeline to left or right.

The web console can't run docker/ros itself, so this tiny host service does it. A
`POST /side/{side}` (left|right):
  1. swaps the served policy MODEL  (recreate the inference container with that side's model)
  2. swaps the VISION node          (stop any system_vision_*, start system_vision_<side>
                                     in the teleop container — Zivid is exclusive, so only
                                     one side runs at a time)
The ARM is just a parameter of the drive script (--side), so nothing to restart there.

Run on the HOST (needs `sudo docker`; passwordless sudo assumed) in the vpi env:
    uvicorn app.side_switcher:app --host 0.0.0.0 --port 8070

Then the frontend's LEFT/RIGHT toggle calls it automatically.
"""
from __future__ import annotations

import os
import shlex
import subprocess
import threading

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware

REPO = os.environ.get("VPI_REPO", "/home/bi/visuomotor-policy-inference")
COMPOSE = os.environ.get("VPI_COMPOSE", "docker-compose.full.yml")
TELEOP = os.environ.get("VPI_TELEOP_CONTAINER", "ros2_teleop_system")
MODELS = {
    "left": "Ngseo/hyundai-uiwang-left-flowmatch",
    "right": "Ngseo/hyundai-uiwang-right-flowmatch",
}

app = FastAPI(title="VPI side switcher", version="1.0.0")
app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"])

_state = {"side": None, "busy": False, "last_model": "", "last_vision": ""}
_lock = threading.Lock()


def _sh(cmd: str, timeout: float = 200.0) -> subprocess.CompletedProcess:
    return subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=timeout)


def _swap_model(side: str) -> str:
    # env.<side> sets VPI_MODEL_ID; recreate just the inference service.
    cmd = (f"cd {shlex.quote(REPO)} && "
           f"sudo docker compose --env-file env.{side} -f {shlex.quote(COMPOSE)} "
           f"up -d --force-recreate inference")
    r = _sh(cmd)
    return "ok" if r.returncode == 0 else f"err: {(r.stderr or r.stdout)[-300:]}"


def _swap_vision(side: str) -> str:
    # 1) STOP any running vision node, in its OWN exec, so the kill pattern can't match
    #    the start command. The `[s]` regex trick also stops pkill matching itself.
    #    Graceful TERM first (lets the node disconnect Zivid cleanly -> no stale session),
    #    then force-kill any leftover.
    stop = ("pkill -TERM -f '[s]ystem_vision_' 2>/dev/null; sleep 4; "
            "pkill -9 -f '[s]ystem_vision_' 2>/dev/null; sleep 2")
    _sh(f"sudo docker exec {shlex.quote(TELEOP)} bash -lc {shlex.quote(stop)}", timeout=30)
    # 2) START the target side (detached, separate exec) — only one runs at a time.
    start = ("source /opt/ros/humble/setup.bash; source /workspace/install/setup.bash; "
             "export ROS_DOMAIN_ID=0; "
             f"exec ros2 run teleop_vision system_vision_{side} > /tmp/vision_{side}.log 2>&1")
    r = _sh(f"sudo docker exec -d {shlex.quote(TELEOP)} bash -lc {shlex.quote(start)}", timeout=30)
    return "started" if r.returncode == 0 else f"err: {(r.stderr or r.stdout)[-300:]}"


@app.get("/side")
def get_side():
    return {"side": _state["side"], "busy": _state["busy"],
            "model": _state["last_model"], "vision": _state["last_vision"], "models": MODELS}


@app.post("/side/{side}")
def set_side(side: str):
    if side not in MODELS:
        raise HTTPException(status_code=400, detail="side must be 'left' or 'right'")
    with _lock:
        if _state["busy"]:
            raise HTTPException(status_code=409, detail="a switch is already in progress")
        _state["busy"] = True
    try:
        model = _swap_model(side)        # blocks ~a few seconds (container recreate)
        vision = _swap_vision(side)      # detached; camera connect takes ~15-30s after
        _state.update(side=side, last_model=model, last_vision=vision)
        return {"side": side, "model": model, "vision": vision,
                "note": "model reloads + cameras connect in the background (~30s)"}
    finally:
        _state["busy"] = False


@app.get("/health")
def health():
    return {"status": "ok"}
