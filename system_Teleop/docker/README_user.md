# Right Vision Docker User Guide

Right Vision 노드를 Docker에서 실행하고, 다른 터미널 또는 다른 Docker 컨테이너에서 ROS 2 토픽을 받는 순서입니다.

## 1. 호스트 장비 확인

Zivid 카메라 네트워크를 먼저 잡습니다.

```bash
sudo ip link set enp4s0 up
sudo ip addr flush dev enp4s0
sudo ip addr add 192.168.4.55/24 dev enp4s0
sudo ip route replace 192.168.4.0/24 dev enp4s0
ZividListCameras
```

right Zivid는 아래처럼 보여야 합니다.

```text
Serial Number: 2051707B, IP Address: 192.168.4.201
```

RealSense D405가 호스트에 보이는지 확인합니다.

```bash
lsusb | grep -i "Intel\|RealSense\|8086"
ls /dev/video*
```

right RealSense serial은 `409122273122`입니다.

## 2. Docker 실행

호스트에서 `system_Teleop` 디렉터리로 이동합니다.

```bash
cd ~/Tesollo/1.Project/1.hyundae/seoul_uiwang/system_Teleop
sudo docker compose up -d --force-recreate
sudo docker compose exec ros2-teleop bash
```

새 이미지가 필요하면 먼저 빌드합니다.

```bash
sudo docker compose build --no-cache --progress=plain ros2-teleop
```

## 3. 컨테이너 초기 확인

컨테이너 안에서 실행합니다.

```bash
cd /workspace
source /opt/ros/humble/setup.bash
[ -f install/setup.bash ] && source install/setup.bash
echo $TELEOP_IMAGE_REV
```

공통 Vision dependency 확인:

```bash
python3 -c "import torch, open3d, pyrealsense2, rfdetr; print('vision deps ok')"
python3 -c "import numpy; print(numpy.__version__)"
```

NVIDIA/OpenCL 확인:

```bash
nvidia-smi
clinfo | grep -E "Number of platforms|Platform Name"
```

## 4. Zivid 수동 설치

현재 Dockerfile은 Zivid SDK/Python binding을 이미지에 포함하지 않습니다. 컨테이너 안에서 필요할 때 설치합니다.

```bash
cd /tmp
wget -O zivid.deb https://downloads.zivid.com/sdk/releases/2.17.2+440b2367-1/u22/amd64/zivid_2.17.2+440b2367-1_amd64.deb
apt-get update
apt-get install -y ./zivid.deb
python3 -m pip install conan
python3 -m pip install zivid==2.17.2
```

확인:

```bash
python3 -c "import zivid; print(zivid.__version__)"
ZividListCameras
```

## 5. Vision 패키지 빌드

컨테이너 안에서 right Vision 실행에 필요한 패키지만 빌드합니다.

```bash
/workspace/docker/build_vision.sh
source /workspace/install/setup.bash
```

## 6. Right Vision 노드 실행

컨테이너 안에서 실행합니다.

```bash
ros2 run teleop_vision system_vision_right
```

정상 로그 예시:

```text
Successfully connected to Right Zivid camera.
Discovered RealSense devices: ['Intel RealSense D405 (SN: 409122273122)']
Successfully connected to Right RealSense camera (SN: 409122273122).
```

이전에 실행한 노드가 카메라를 잡고 있으면 종료합니다.

```bash
pkill -9 -f system_vision_right
pkill -9 -f system_vision_left
```

## 7. 같은 컨테이너에서 토픽 수신 확인

새 터미널에서 컨테이너에 접속합니다.

```bash
cd ~/Tesollo/1.Project/1.hyundae/seoul_uiwang/system_Teleop
sudo docker compose exec ros2-teleop bash
cd /workspace
source /opt/ros/humble/setup.bash
source install/setup.bash
```

토픽 목록:

```bash
ros2 topic list | grep system_right
```

프레임 수신률:

```bash
ros2 topic hz /system_right/d405_rgb
ros2 topic hz /system_right/zivid_rgb
```

메시지 타입:

```bash
ros2 topic info /system_right/d405_rgb
ros2 topic info /system_right/zivid_rgb
```

header만 확인:

```bash
ros2 topic echo --once /system_right/d405_rgb --field header
ros2 topic echo --once /system_right/zivid_rgb --field header
```

## 8. 다른 Docker 컨테이너에서 수신

다른 컨테이너도 같은 PC에서 ROS 2 토픽을 받으려면 host network와 같은 ROS domain을 사용합니다.

```yaml
network_mode: host
environment:
  - ROS_DOMAIN_ID=0
```

다른 컨테이너 안에서:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
ros2 topic list | grep system_right
ros2 topic hz /system_right/d405_rgb
ros2 topic hz /system_right/zivid_rgb
```

주의:

- 다른 컨테이너가 bridge network면 ROS 2 DDS discovery가 안 될 수 있습니다.
- `ROS_DOMAIN_ID`가 다르면 서로 보이지 않습니다.
- `ROS_LOCALHOST_ONLY=1`이 설정되어 있으면 다른 네트워크 namespace와 통신이 막힐 수 있습니다.

## 9. 녹화 테스트

Right Vision 노드가 실행 중인 상태에서 다른 터미널에서 실행합니다.

```bash
ros2 topic pub --once /system_right/start_recording system_interface/msg/StartRecording "{start_record: true}"
sleep 5
ros2 topic pub --once /system_right/start_recording system_interface/msg/StartRecording "{start_record: false}"
ls -lh /workspace/Record/right/videos
```

## 10. 자주 쓰는 복구 명령

카메라가 busy일 때:

```bash
pkill -9 -f system_vision_right
pkill -9 -f system_vision_left
```

RealSense가 호스트에는 보이는데 컨테이너에서 안 보일 때:

```bash
sudo docker compose down
sudo docker compose up -d --force-recreate
sudo docker compose exec ros2-teleop bash
```

NumPy/cv_bridge 에러가 날 때:

```bash
python3 -m pip install "numpy<2" "setuptools==58.2.0"
/workspace/docker/build_vision.sh
```
