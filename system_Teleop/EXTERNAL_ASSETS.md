# system_Teleop — external assets (NOT in git)

This `system_Teleop/` is the teleop ROS2 workspace (vision node, hand drivers, arm
OpenStream client, recorders, etc.) that the inference/replay scripts in this repo depend on.
It was the friend's workspace; the **source code + the Manus SDK `.so` binaries are committed
here**, but the large weights / datasets / build artifacts are **excluded** (the full original
tree was 87 GB; the code is ~9 MB, the rest is the assets below).

Original location: `~/Tesollo/1.Project/1.hyundae/seoul_uiwang/system_Teleop`.

## Excluded — model weights (re-download / re-train; not needed for the policy pipeline)
| path | size | what / needed when |
|---|---|---|
| `src/Vision_/sam3/weights/sam3_best.pt` | 9.4 GB | SAM3 segmentation. **Not needed** — the vision node runs with `VISION_SKIP_SEG=1` (RGB-only). |
| `src/Vision_/segmentation/weights/rf_detr_best.pth` | 148 MB | RF-DETR segmentation. **Not needed** — skipped by `VISION_SKIP_SEG=1`. |
| `src/System_/lerobot_system/lerobot/outputs/train/*/checkpoints/.../model.safetensors` | ~3 GB | the friend's locally-trained LeRobot policies. **Not used** here — our inference server loads the policy from the HF Hub (`Ngseo/hyundai-uiwang-{left,right}-flowmatch`). |

## Excluded — other heavy / regenerable
| path | size | note |
|---|---|---|
| `Record/`, `Record.zip` | 71 GB | recorded episode logs (`StateRecordBin`). Data, not source. |
| `src/System_/lerobot_system/lerobot/.git/` (+ LFS) | ~1.4 GB | the vendored LeRobot fork's own git history/LFS. The **source files are kept**; only its history is dropped. |
| `build/`, `install/`, `log/` | ~0.6 GB | colcon build artifacts — regenerate with `colcon build`. |
| `__pycache__/`, `*.pyc` | — | python bytecode. |

## Excluded — Manus SDK binaries (exceed GitHub's 100 MB/file limit)
These are **not used by the inference / replay / closed-loop pipeline** (we drive the arm +
Inspire/DG5F hands, not the Manus glove). Re-fetch from the Manus SDK install if you build the
Manus packages, and drop them back at these paths:
| path | size |
|---|---|
| `src/Manus_/ManusSDK/lib/libManusSDK.so` | 138 MB |
| `src/Manus_/ManusSDK/lib/libManusSDK_Integrated.so` | 112 MB |
| `src/Delto_/dg5f_driver/include/libManusSDK.so` | 137 MB (a duplicate of the first) |

## Included (committed)
- All ROS2 package **source** (`src/**/*.py`, `*.cpp/*.hpp`, `*.xml`, `*.yaml`, `*.xacro`, launch, urdf, meshes, configs).
- `Dockerfile`, `docker-compose.yml` (with the `left`/`right` profiles + zivid/conan/ros2_control), `docker/`.
- Small vendored libs like `libdelto_gripper_helper.so` (40 KB, needed for the DG5F hand). The
  big Manus `.so` are excluded — see above.

## To run on a fresh machine
1. Build the image: `docker compose build` (installs Zivid SDK, conan, ros2_control, pyserial, requests, rfdetr, etc.).
2. Bring a side up: `docker compose --profile right up -d` (or use `../up.sh right`).
3. If you actually need SAM3/RF-DETR segmentation (we don't, we skip it), re-fetch those `.pt`/`.pth` weights; otherwise leave `VISION_SKIP_SEG=1`.
4. The policy itself comes from the HF Hub at inference-server start — nothing to fetch here.
