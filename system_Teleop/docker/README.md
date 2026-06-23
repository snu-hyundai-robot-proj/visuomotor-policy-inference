# Teleop System Docker

ROS 2 Humble 기반 Teleop 워크스페이스를 Docker/Compose로 실행하기 위한 안내입니다.

이 문서는 `seoul_uiwang/system_Teleop/docker-compose.yml`과 `seoul_uiwang/system_Teleop/Dockerfile` 기준으로 작성되었습니다.

## 구성

- Base image: `osrf/ros:humble-desktop`
- Workspace: 컨테이너 내부 `/workspace`
- Compose service: `ros2-teleop`
- Container name: `ros2_teleop_system`
- Image name: `ros2-teleop:latest`
- Network: `host`
- ROS_DOMAIN_ID: 기본값 `0`
- GUI: host X11 socket mount
- Camera/USB: `/dev/bus/usb` mount, `/dev/videoN` optional mount
- Shared memory: `/dev/shm` mount
- Build artifacts: Docker named volumes for `/workspace/build`, `/workspace/install`, `/workspace/log`

## 사전 준비

호스트에 Docker와 Docker Compose가 설치되어 있어야 합니다.

GUI/RViz/OpenCV 창을 컨테이너에서 띄우려면 X11 접근을 허용합니다.

```bash
xhost +local:docker
```

카메라 장치가 보이는지 확인합니다.

```bash
ls /dev/video*
lsusb
```

`ls /dev/video*`에서 `No such file or directory`가 나오면 현재 호스트에 V4L2 카메라 노드가 없는 상태입니다. USB 카메라가 연결되어 있지 않거나, RealSense/Zivid 장치가 OS에서 카메라로 인식되지 않은 상태일 수 있습니다.

현재 Vision 노드는 Zivid와 RealSense를 사용합니다. 장치가 정상 연결되면 `lsusb`에서 해당 카메라가 보여야 하고, RealSense는 환경에 따라 `/dev/video0`, `/dev/video1` 같은 노드도 생성됩니다.

RealSense를 사용하는 경우 호스트 쪽 udev rule 또는 권한 설정이 필요할 수 있습니다. Zivid를 사용하는 경우 호스트/컨테이너에 Zivid SDK와 장치 접근 권한이 필요합니다.

## 빌드

반드시 `system_Teleop` 디렉터리에서 실행합니다.

```bash
cd seoul_uiwang/system_Teleop
docker compose build
```

이미지 빌드는 OS/ROS 개발 도구까지만 설치합니다. ROS workspace 빌드는 컨테이너 실행 후 `/workspace`에서 수행합니다.

캐시 없이 다시 빌드하려면:

```bash
docker compose build --no-cache
```

## 실행

컨테이너를 백그라운드로 실행합니다.

```bash
cd seoul_uiwang/system_Teleop
docker compose up -d
```

컨테이너 쉘에 접속합니다.

```bash
docker compose exec ros2-teleop bash
```

컨테이너 안에서는 기본적으로 다음 환경이 `.bashrc`에 등록되어 있습니다.

```bash
source /opt/ros/humble/setup.bash
[ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash
```

새 쉘에서 ROS 패키지가 보이지 않으면 직접 다시 source 합니다.

```bash
source /opt/ros/humble/setup.bash
[ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash
```

처음 실행한 컨테이너에서는 아직 `/workspace/install/setup.bash`가 없을 수 있습니다. 아래 워크스페이스 빌드를 먼저 수행합니다.

## Vision 단독 빌드

Vision 노드만 확인하려면 전체 workspace에 `rosdep install --from-paths src`를 실행하지 않습니다. 그러면 Robot/MoveIt/LeRobot/Tracker 의존성까지 모두 설치하려고 해서 시간이 오래 걸리고, 현재 장비에 필요 없는 패키지에서 실패할 수 있습니다.

컨테이너 안에서:

```bash
/workspace/docker/build_vision.sh
```

스크립트는 내부적으로 `setuptools==58.2.0`을 맞춘 뒤 `system_interface`와 `teleop_vision`만 빌드합니다.

Vision 노드 실행:

```bash
ros2 run teleop_vision system_vision_left
```

다른 쉘에서 토픽 확인:

```bash
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
ros2 topic hz /system_left/d405_rgb
ros2 topic hz /system_left/zivid_rgb
```

## 전체 워크스페이스 빌드

컨테이너 안에서 ROS dependency를 설치하고 workspace를 빌드합니다.

```bash
cd /workspace
apt-get update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

전체 빌드에서 아래처럼 rosdep key를 못 찾는 경우가 있습니다.

```text
Cannot locate rosdep definition for [ament_python]
Cannot locate rosdep definition for [OpenCV]
Cannot locate rosdep definition for [tmm_msgs]
```

`ament_python`은 ROS package build type이고, `OpenCV`는 이미 Dockerfile에서 apt 패키지로 설치합니다. `tmm_msgs`는 현재 workspace에 없으면 해당 패키지를 빌드 제외하거나 의존 패키지를 추가해야 합니다. 우선 전체 빌드를 진행해야 한다면 다음처럼 skip할 수 있습니다.

```bash
rosdep install --from-paths src --ignore-src -r -y --skip-keys "ament_python OpenCV tmm_msgs"
```

`docker-compose.yml`은 `/workspace/build`, `/workspace/install`, `/workspace/log`를 Docker named volume으로 분리합니다. 호스트의 오래된 `build/`, `install/`, `log/`가 컨테이너 빌드 결과를 덮지 않게 하기 위한 설정입니다.

소스 수정 후에도 컨테이너 안에서 다시 빌드합니다.

```bash
cd /workspace
colcon build --symlink-install
source install/setup.bash
```

특정 패키지만 빌드하려면:

```bash
colcon build --symlink-install --packages-select system_interface teleop_vision
source install/setup.bash
```

## Vision 런타임 패키지

Dockerfile은 Vision 노드 실행에 필요한 공통 Python 패키지를 이미지에 포함합니다. Zivid SDK/Python binding은 이미지 빌드 안정성을 위해 제외하고, 컨테이너 안에서 필요할 때 직접 설치합니다.

포함되는 주요 항목:

- PyTorch CPU wheel: `torch`, `torchvision`, `torchaudio`
- Vision Python packages: `numpy<2`, `scipy`, `scikit-image`, `pillow`, `open3d`, `pyyaml`, `tqdm`, `rfdetr`, `pyrealsense2`
- OpenCL tools/ICD: `clinfo`, `ocl-icd-*`, `intel-opencl-icd`, NVIDIA ICD vendor file

새 Dockerfile 내용을 반영하려면 이미지를 다시 빌드합니다.

```bash
docker compose build --no-cache
docker compose up -d --force-recreate
```

`opencv-python`은 Dockerfile에서 apt 패키지 `python3-opencv`로 설치합니다. `cv2` import가 되는지 확인합니다.

```bash
python3 -c "import cv2; print(cv2.__version__)"
```

설치 확인:

```bash
python3 -c "import torch, open3d, pyrealsense2, rfdetr; print('vision deps ok')"
```

Zivid SDK/Python binding 수동 설치:

```bash
cd /tmp
wget https://downloads.zivid.com/sdk/releases/2.17.2+440b2367-1/u22/amd64/zivid_2.17.2+440b2367-1_amd64.deb
apt-get update
apt-get install -y ./zivid_2.17.2+440b2367-1_amd64.deb
python3 -m pip install conan
python3 -m pip install zivid==2.17.2
python3 -c "import zivid; print(zivid.__version__)"
```

ROS Humble의 `cv_bridge`는 NumPy 2.x와 ABI가 맞지 않을 수 있습니다. 아래 오류가 보이면 NumPy를 1.x로 낮춘 뒤 Vision 패키지를 다시 빌드합니다.

```text
A module that was compiled using NumPy 1.x cannot be run in NumPy 2.x
AttributeError: _ARRAY_API not found
```

```bash
python3 -m pip install "numpy<2" "setuptools==58.2.0"
/workspace/docker/build_vision.sh
```

장치 확인은 USB mount와 장비 연결이 되어 있는 상태에서 수행합니다.

Zivid 네트워크 카메라가 검색되지 않으면 먼저 호스트 네트워크를 확인합니다. Docker compose는 `network_mode: host`를 사용하므로, 호스트에서 안 보이는 Zivid는 컨테이너에서도 보이지 않습니다.

```bash
ip -br addr
ip route
ip neigh show
```

현재 프로젝트 ReadMe 기준 로봇/장비망은 `192.168.4.0/24`를 사용합니다. 유선 NIC가 `enp4s0`이고 IP가 비어 있으면 다음처럼 임시 IP와 route를 설정합니다.

```bash
sudo ip link set enp4s0 up
sudo ip addr flush dev enp4s0
sudo ip addr add 192.168.4.55/24 dev enp4s0
sudo ip route replace 192.168.4.0/24 dev enp4s0
ip -br addr
ip route
ZividListCameras
```

정상적으로 잡히면 `ZividListCameras`에서 left/right 카메라가 아래처럼 보여야 합니다.

```text
Serial Number: 23352865, IP Address: 192.168.4.200
Serial Number: 2051707B, IP Address: 192.168.4.201
```

공유기 DHCP 대역이 `192.168.0.0/24`라면 Zivid도 같은 대역 IP를 받았을 수 있습니다. 이 경우 `ip neigh show`에서 새 장치 IP를 확인하거나 공유기 관리자 페이지의 DHCP client list에서 Zivid를 찾습니다.

Zivid SDK에서 검색:

```bash
ZividListCameras
python3 - <<'PY'
import zivid
app = zivid.Application()
cameras = app.cameras()
print(f"found cameras: {len(cameras)}")
for camera in cameras:
    print(camera)
PY
```

Zivid 실행 중 OpenCL platform 에러가 나오면 컨테이너에서 GPU/OpenCL이 보이지 않는 상태입니다.

```text
RuntimeError: An OpenCL error occurred: Failed to get platforms
CL_PLATFORM_NOT_FOUND_KHR
```

먼저 호스트와 컨테이너에서 각각 확인합니다.

```bash
clinfo | grep -E "Number of platforms|Platform Name"
```

컨테이너에 `clinfo`가 없거나 Intel GPU/OpenCL ICD가 없다면:

```bash
apt-get update
apt-get install -y clinfo ocl-icd-libopencl1 ocl-icd-opencl-dev intel-opencl-icd
clinfo | grep -E "Number of platforms|Platform Name"
```

호스트에서는 platform이 보이는데 컨테이너에서는 안 보이면 GPU device/runtime이 컨테이너에 전달되지 않은 것입니다. Intel/AMD iGPU는 `/dev/dri` mount가 필요하고, 현재 compose는 `/dev/dri`를 전달합니다.

호스트 `clinfo`에서 `NVIDIA CUDA`가 보이고 컨테이너 `clinfo`가 `Number of platforms 0`이면 NVIDIA container runtime이 필요합니다. 호스트에 `nvidia-container-toolkit`을 설치한 뒤 `docker-compose.yml`의 `gpus: all`, `NVIDIA_VISIBLE_DEVICES=all`, `NVIDIA_DRIVER_CAPABILITIES=all` 설정으로 컨테이너를 다시 생성합니다.

```bash
docker compose down
docker compose up -d --force-recreate
docker compose exec ros2-teleop bash
clinfo | grep -E "Number of platforms|Platform Name"
```

설정이 실제 컨테이너에 붙었는지 확인:

```bash
docker inspect ros2_teleop_system --format '{{json .HostConfig.DeviceRequests}}'
```

컨테이너 안에서 NVIDIA runtime이 붙었는지 확인:

```bash
nvidia-smi
ls /etc/OpenCL/vendors
```

`nvidia-smi`는 되는데 `/etc/OpenCL/vendors`가 없거나 `clinfo`가 여전히 0 platform이면 NVIDIA OpenCL ICD vendor file이 없는 상태입니다. 현재 컨테이너에서 바로 복구하려면:

```bash
apt-get update
apt-get install -y clinfo ocl-icd-libopencl1 ocl-icd-opencl-dev
mkdir -p /etc/OpenCL/vendors
echo "libnvidia-opencl.so.1" > /etc/OpenCL/vendors/nvidia.icd
ldconfig
clinfo | grep -E "Number of platforms|Platform Name"
```

`libnvidia-opencl.so.1`가 실제로 보이는지도 확인할 수 있습니다.

```bash
ldconfig -p | grep libnvidia-opencl
```

`clinfo`가 최소 1개 platform을 보여준 뒤 다시 확인합니다.

```bash
python3 -c "import zivid; app = zivid.Application(); print('zivid ok')"
```

## RealSense 확인

Vision 노드는 RealSense serial을 고정해서 연결합니다.

- left: `409122273797`
- right: `409122273122`

호스트에서 장치가 보이는지 먼저 확인합니다.

```bash
lsusb | grep -i "Intel\\|RealSense"
ls /dev/video*
```

컨테이너 안에서 pyrealsense2가 장치를 보는지 확인합니다.

```bash
python3 - <<'PY'
import pyrealsense2 as rs

ctx = rs.context()
devices = ctx.query_devices()
print(f"found RealSense devices: {len(devices)}")
for device in devices:
    print(device.get_info(rs.camera_info.name), device.get_info(rs.camera_info.serial_number))
PY
```

장치가 보이는데 serial이 위 값과 다르면 `vision_node_left.py`, `vision_node_right.py`의 `rs_configs` 값을 실제 serial로 수정한 뒤 다시 빌드합니다.

```bash
/workspace/docker/build_vision.sh
```

호스트에서는 보이는데 컨테이너에서 0개면 `/dev/bus/usb:/dev/bus/usb:rw` mount가 유지되어 있는지 확인하고, 필요하면 compose에서 `privileged: true`로 테스트합니다.

D405가 호스트 `lsusb`에는 보이지만 컨테이너 `pyrealsense2`에서 0개로 나오면 컨테이너를 재생성합니다. USB 장치 hot-plug 후 기존 컨테이너가 새 device node 권한을 제대로 못 보는 경우가 있습니다.

```bash
sudo docker compose down
sudo docker compose up -d --force-recreate
sudo docker compose exec ros2-teleop bash
```

컨테이너 안에서 다시 확인:

```bash
lsusb | grep -i "Intel\\|RealSense\\|8086"
python3 - <<'PY'
import pyrealsense2 as rs
ctx = rs.context()
devices = ctx.query_devices()
print(f"found RealSense devices: {len(devices)}")
for device in devices:
    print(device.get_info(rs.camera_info.name), device.get_info(rs.camera_info.serial_number))
PY
```

현재 compose는 RealSense 디버깅 편의를 위해 `/dev:/dev`와 `privileged: true`를 사용합니다. 운영 환경에서는 필요한 장치만 좁혀서 전달하는 방식으로 되돌리는 것을 권장합니다.

## 주요 실행 예시

Vision 노드:

```bash
ros2 run teleop_vision system_vision_left
ros2 run teleop_vision system_vision_right
```

System UI:

```bash
ros2 run system_ui system_ui_node
```

LeRobot system bridge:

```bash
ros2 run lerobot_system lerobot_system
ros2 run lerobot_system lerobot_system_left
ros2 run lerobot_system lerobot_system_right
```

LeRobot launch:

```bash
ros2 launch lerobot_system lerobot_system.launch.py
```

Vive tracker:

```bash
ros2 run vive_tracker_core tracker_core
ros2 launch vive_tracker_bringup vive_tracker_bringup.launch.py
```

HDR bringup:

```bash
ros2 launch hdr_bringup hdr_control.launch.py
ros2 launch hdr_bringup hdr_moveit.launch.py
```

Delto DG5F:

```bash
ros2 launch dg5f_driver dg5f_left_driver.launch.py
ros2 launch dg5f_driver dg5f_left_pid_controller.launch.py
```

Force/Torque sensor:

```bash
ros2 launch net_ft_driver net_ft_broadcaster.launch.py
ros2 launch net_ft_driver dual_axia_reader.launch.py
```

## ROS 네트워크 설정

현재 compose는 host network를 사용하므로 호스트와 컨테이너가 같은 ROS 네트워크를 사용합니다.

기본 `ROS_DOMAIN_ID`는 `0`입니다. 다른 장비 또는 호스트 ROS와 맞춰야 하면 `docker-compose.yml`의 값을 수정하거나 실행 시 override 합니다.

```bash
ROS_DOMAIN_ID=10 docker compose up -d
```

컨테이너 안에서 확인:

```bash
echo $ROS_DOMAIN_ID
ros2 node list
ros2 topic list
```

## 장치 권한

compose는 기본적으로 다음 장치를 컨테이너에 전달합니다.

- `/dev/bus/usb`
- `/dev/dri`

`/dev/video0`, `/dev/video1` 같은 장치가 실제로 존재하는 경우에만 `docker-compose.yml`에 명시적으로 추가합니다.

```yaml
volumes:
  - /dev/video0:/dev/video0:rw
  - /dev/video1:/dev/video1:rw
```

`/dev/video*` 와일드카드 bind mount는 Docker Compose에서 안정적으로 처리되지 않으므로 사용하지 않습니다.

카메라나 GPU 장치 접근이 안 되면 먼저 호스트에서 장치가 잡히는지 확인하고, 필요하면 compose의 `privileged` 값을 `true`로 바꿔 테스트합니다.

```yaml
privileged: true
```

운영 환경에서는 필요한 장치만 `devices`/`volumes`에 명시하는 방식을 권장합니다.

## GUI 문제 해결

`cannot connect to X server` 또는 RViz/OpenCV 창이 뜨지 않으면 호스트에서:

```bash
echo $DISPLAY
xhost +local:docker
```

컨테이너 안에서:

```bash
echo $DISPLAY
ls /tmp/.X11-unix
```

그래도 안 되면 compose의 `DISPLAY=${DISPLAY}`와 `/tmp/.X11-unix:/tmp/.X11-unix:rw` mount가 유지되어 있는지 확인합니다.

## 빌드 문제 해결

`teleop_vision` 빌드 중 `option --editable not recognized`가 나오면 컨테이너 안의 `setuptools`가 ROS 2 Humble의 `ament_python` 빌드 흐름과 맞지 않는 상태입니다.

```text
error: option --editable not recognized
```

현재 컨테이너에서 바로 복구하려면:

```bash
python3 -m pip install "setuptools==58.2.0"
rm -rf build/teleop_vision install/teleop_vision log
colcon build --symlink-install --packages-up-to teleop_vision
source install/setup.bash
```

Dockerfile에서는 `setuptools`를 pip로 upgrade하지 않고 Ubuntu/ROS 패키지 버전을 사용합니다.

PyTorch 설치 중 `Cannot uninstall sympy 1.9`가 나오면 Ubuntu/ROS 이미지에 apt/distutils로 설치된 `sympy`를 pip가 제거하려고 한 경우입니다.

```text
error: uninstall-distutils-installed-package
Cannot uninstall sympy 1.9
```

컨테이너에서는 pip 패키지를 `/usr/local`에 새로 올리면 되므로 `--ignore-installed`를 붙여 설치합니다.

```bash
python3 -m pip install --ignore-installed torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu
```

Zivid Python binding 설치 중 `CMake was unable to find a build program corresponding to "Ninja"`가 나오면 `ninja-build`가 없는 상태입니다. Dockerfile에는 `ninja-build`가 포함되어 있어야 합니다. Zivid Python wheel build에는 Python headers도 필요하므로 `python3-dev`도 포함되어 있어야 합니다.

`opencv-python`을 찾지 못한다는 에러가 나오면 PyTorch CPU wheel 인덱스가 일반 pip 패키지 설치까지 적용된 경우입니다.

```text
ERROR: Could not find a version that satisfies the requirement opencv-python
ERROR: No matching distribution found for opencv-python
```

Dockerfile에서 `torch`, `torchvision`, `torchaudio` 설치와 `opencv-python`, `numpy` 등 일반 PyPI 패키지 설치가 별도 `pip install` 명령으로 분리되어 있어야 합니다.

`rosdep init`에서 이미 초기화되어 있다는 에러가 나오면 base image에 rosdep source list가 이미 들어있는 경우입니다.

```text
ERROR: default sources list file already exists:
  /etc/ros/rosdep/sources.list.d/20-default.list
```

Dockerfile은 해당 파일이 없을 때만 `rosdep init`을 실행하고, 이후 `rosdep update`만 수행하도록 되어 있어야 합니다.

`load build context` 단계에서 수 GB 이상 전송되거나 `Killed`가 나오면 Docker build context가 너무 큰 상태입니다.

```text
transferring context: 15.67GB
Killed
```

`system_Teleop/.dockerignore`에서 `Record/`, `build/`, `install/`, `log/`, 모델 weight, `.git`, archive 파일을 제외해야 합니다. Compose 실행 시에는 `.:/workspace`가 mount되므로, build context에서 제외된 런타임 데이터도 호스트 workspace에는 그대로 남아 있습니다.

`/opt/ros/humble/setup.bash: Bad substitution`이 나오면 Docker `RUN`이 `/bin/sh`로 실행된 경우입니다.

```text
/bin/sh: 1: /opt/ros/humble/setup.bash: Bad substitution
```

Dockerfile에 `SHELL ["/bin/bash", "-c"]`가 있어야 ROS `setup.bash`를 정상적으로 source할 수 있습니다.

## 종료와 정리

컨테이너 종료:

```bash
docker compose down
```

이미지까지 다시 만들고 싶을 때:

```bash
docker compose down
docker rmi ros2-teleop:latest
docker compose build --no-cache
```

## 참고

- `docker-compose.yml`은 workspace 전체를 `.:/workspace`로 mount합니다.
- 이미지 빌드 시점에도 `COPY . /workspace/`가 수행되지만, compose 실행 중에는 호스트 디렉터리 mount가 우선 적용됩니다.
- 따라서 소스 수정은 호스트에서 바로 반영되고, C++/ROS 인터페이스 변경 후에는 컨테이너 안에서 `colcon build --symlink-install`을 다시 실행해야 합니다.
