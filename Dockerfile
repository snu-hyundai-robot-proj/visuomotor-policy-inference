# syntax=docker/dockerfile:1
# Visuomotor Policy Inference server.
# Single image runs on CPU by default; set VPI_DEVICE=cuda + an NVIDIA runtime for GPU.
#
# Size note: most of the image is the mandatory CUDA torch stack — torch eagerly
# preloads ALL its bundled nvidia/* CUDA libs at import, so none can be removed while
# keeping GPU support. We trim the packages that the *serving path never imports*
# (verified: rerun, wandb, opencv, imageio-ffmpeg, cmake) plus bytecode/test files.
FROM python:3.11-slim

ENV PYTHONUNBUFFERED=1 \
    HF_HOME=/cache/huggingface \
    VPI_MODEL_ID=Ngseo/hyundai-uiwang-left-flowmatch \
    VPI_DEVICE=cpu

# Runtime libs: git (pip installs lerobot from the fork), ffmpeg/libgl/libglib (av).
RUN apt-get update && apt-get install -y --no-install-recommends \
        git ffmpeg libgl1 libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /srv
COPY requirements.txt ./

# One layer: install (with a build toolchain for the evdev wheel), then prune unused
# packages + bytecode, then purge the toolchain — so the savings land in this layer.
# The pip cache mount keeps the ~5 GB of wheels out of the image and speeds rebuilds.
RUN --mount=type=cache,target=/root/.cache/pip \
    apt-get update && apt-get install -y --no-install-recommends build-essential linux-libc-dev \
 && pip install --upgrade pip && pip install -r requirements.txt \
 && pip uninstall -y rerun-sdk wandb opencv-python-headless imageio-ffmpeg cmake gym-pusht pymunk || true \
 && apt-get purge -y build-essential linux-libc-dev && apt-get autoremove -y \
 && rm -rf /var/lib/apt/lists/* \
 && find /usr/local/lib/python3.11/site-packages \( -name '__pycache__' -o -name 'tests' -o -name 'test' \) \
        -type d -prune -exec rm -rf {} + \
 && find /usr/local/lib/python3.11/site-packages -name '*.pyc' -delete

COPY app ./app
COPY examples ./examples

EXPOSE 8000

# The model (~1.1 GB) is downloaded from the Hub on first startup into HF_HOME
# (persist it with a volume — see docker-compose.yml). Gated model -> pass HF_TOKEN.
CMD ["uvicorn", "app.server:app", "--host", "0.0.0.0", "--port", "8000"]
