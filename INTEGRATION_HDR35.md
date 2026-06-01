# HDR35 + DG5F 텔레오퍼레이션에 Visuomotor 정책 서버 연동 가이드

> 목표: `visuomotor-policy-inference` HTTP 서버(Hyundai Uiwang FlowMatch 정책)를
> **중간 폴리시**로 사용해서 **Zivid·RealSense 카메라를 입력**으로 받고
> **HDR35 로봇팔 + DG5F 그리퍼에 action을 출력**한다.

---

## 1. 핵심 아이디어

원하는 "중간 폴리시 노드"는 **이미 거의 다 만들어져 있다.**
`system_Teleop/src/System_/lerobot_system/` 노드가 바로
**상태+카메라 구독 → 정책 추론 → arm/hand 액션 발행**을 하는 노드다.

차이점은 단 하나:

| | 기존 `lerobot_system` | 이번에 원하는 것 |
|---|---|---|
| 정책 위치 | 노드 안에서 LeRobot 모델을 **직접 로드** | 별도 프로세스의 **HTTP 서버**(`visuomotor-policy-inference`) |
| 모델 | 동일 (`hyundai-uiwang-left-flowmatch`) | 동일 |

즉, **정책 호출부만 HTTP 클라이언트로 바꾸면** 나머지 배선(상태 조립, 액션 분할,
단위 변환, 안전 클램프, 출력 게이트)은 그대로 재사용된다.

### 데이터 흐름

```
[Zivid]      ──/system_left/camera/front/rgb──┐
[RealSense]  ──/system_left/camera/wrist/rgb──┤
                                              ├─► lerobot_system 노드
[HDR35 6 joint] ┐                             │   (HttpPolicyRunner)
[DG5F 20 joint] ┴── /system_left/frame_aligned_state ─┘   │
                                                          │ POST /predict
                                              ┌───────────┴─────────────┐
                                              │ visuomotor 서버 :8000   │
                                              │  front_rgb, wrist_rgb,  │
                                              │  state[26] → action[26] │
                                              └───────────┬─────────────┘
                          arm[0:6] (rad→deg) ─► /robot/joint_target_deg ─► HDR35
                          hand[6:26] (rad)    ─► /dg5f_left/lj_dg_pospid/reference ─► DG5F
```

### 입출력 규격 (서버 ↔ 모델)

- **state (26)** = `robot_joint`(6, arm 관절) + `gripper_joint`(20, hand 관절). 학습 데이터는 **라디안**.
- **action (26)** = arm 6 + hand 20, 30 Hz 목표값. arm은 라디안, hand도 라디안.
- **카메라**: `front_rgb`(Zivid, 씬 카메라), `wrist_rgb`(RealSense, 손목 카메라). 해상도는 서버가 내부에서 리사이즈하므로 무관.

> ⚠️ 모델은 **left 전용**(`hyundai-uiwang-left-flowmatch`)이다. 오른팔은 별도 모델/서버 필요.

---

## 2. 사전 준비 — 정책 서버 띄우기

```bash
cd ~/Tesollo/visuomotor-policy-inference

# (GPU 사용 시) docker-compose.yml의 deploy GPU 블록 주석 해제 + 아래 env
# export VPI_DEVICE=cuda

docker compose up --build       # http://localhost:8000, 첫 실행 시 모델(~1.1GB) 다운로드

# 헬스체크
curl localhost:8000/health
curl localhost:8000/info        # state_dim=26, action_dim=26, cameras 확인
```

Docker 없이 dev로 띄우려면:

```bash
pip install -r requirements.txt          # lerobot 포크 설치됨 (PyPI lerobot 아님!)
uvicorn app.server:app --host 0.0.0.0 --port 8000
```

> ROS 노드가 다른 PC에서 돈다면 `localhost` 대신 서버 PC의 IP를 쓴다.

---

## 3. 구현 (총 3 스텝)

작업 위치: `~/Tesollo/1.Project/1.hyundae/seoul_uiwang/system_Teleop/`

### Step 1 — 카메라 Image 토픽 발행 추가  ⚠️ 유일한 실제 공백

현재 `Vision_/teleop_vision/vision_node_left.py` 는 Zivid·RealSense 프레임을
**디스크에만 저장**하고 ROS 토픽으로 **발행하지 않는다.** 실시간 추론에 넣으려면
`sensor_msgs/Image` 퍼블리셔를 추가해야 한다.

`vision_node_left.py` (오른쪽이면 `_right`)에 추가:

```python
# --- import 부 ---
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# --- __init__ 안, 노드 생성 직후 ---
self.bridge = CvBridge()
self.front_rgb_pub = self.create_publisher(Image, "/system_left/camera/front/rgb", 1)
self.wrist_rgb_pub = self.create_publisher(Image, "/system_left/camera/wrist/rgb", 1)

# --- Zivid 2D 프레임을 얻은 직후 (RGB numpy, HxWx3 uint8) ---
def _publish_front(self, rgb: "np.ndarray"):
    msg = self.bridge.cv2_to_imgmsg(rgb, encoding="rgb8")
    msg.header.stamp = self.get_clock().now().to_msg()
    self.front_rgb_pub.publish(msg)

# --- RealSense 컬러 프레임을 얻은 직후 ---
def _publish_wrist(self, color_bgr: "np.ndarray"):
    # RealSense는 BGR8이므로 인코딩만 맞춰주면 정책 노드가 rgb8로 변환해 받는다
    msg = self.bridge.cv2_to_imgmsg(color_bgr, encoding="bgr8")
    msg.header.stamp = self.get_clock().now().to_msg()
    self.wrist_rgb_pub.publish(msg)
```

기존 캡처 루프(Zivid 2D capture, RealSense `rs_buffer`에 넣는 지점)에서
`self._publish_front(...)`, `self._publish_wrist(...)`를 호출하면 된다.
프레임 레이트는 정책 fps(30Hz) 이상이면 충분하다.

> 정책 노드는 `imgmsg_to_cv2(msg, desired_encoding="rgb8")`로 받으므로
> Zivid는 `rgb8`, RealSense는 `bgr8`로 발행해도 자동 변환된다.

확인:
```bash
ros2 topic hz /system_left/camera/front/rgb
ros2 topic hz /system_left/camera/wrist/rgb
```

---

### Step 2 — HTTP 백엔드 러너 추가

`lerobot_system/lerobot_system/` 폴더에 **새 파일** `http_policy_runner.py` 생성.
기존 `LeRobotPolicyRunner`와 **동일한 인터페이스**(`select_action`, `state_dim`,
`action_dim`, `image_features`, `policy_type`)를 제공하므로 노드는 둘을 똑같이 다룬다.

```python
# lerobot_system/lerobot_system/http_policy_runner.py
"""visuomotor-policy-inference HTTP 서버를 호출하는 정책 러너.

LeRobotPolicyRunner와 동일한 인터페이스를 제공해서 node.py가 그대로 사용한다.
서버 API: GET /info, POST /reset, POST /predict {front_rgb, wrist_rgb, state} -> {action}
"""
from __future__ import annotations

import base64
import io
from typing import Any, Dict

import numpy as np
import requests
from PIL import Image as PILImage

# node.py가 observation에 넣는 키 (정책/서버의 카메라 키와 동일해야 함)
FRONT_KEY = "observation.images.front_rgb"
WRIST_KEY = "observation.images.wrist_rgb"
STATE_KEY = "observation.state"


def _encode_png(arr: np.ndarray) -> str:
    buf = io.BytesIO()
    PILImage.fromarray(np.asarray(arr, dtype=np.uint8)).save(buf, format="PNG")
    return base64.b64encode(buf.getvalue()).decode("ascii")


class HttpPolicyRunner:
    """원격 추론 서버를 감싸는 러너. node.py 입장에선 로컬 러너와 동일하게 보인다."""

    def __init__(self, server_url: str, timeout_sec: float = 5.0, reset_on_init: bool = True):
        self.server_url = server_url.rstrip("/")
        self.timeout = timeout_sec
        self.policy_type = "http"

        info = requests.get(f"{self.server_url}/info", timeout=self.timeout).json()
        self.state_dim = int(info["state_dim"])      # 26
        self.action_dim = int(info["action_dim"])    # 26
        self.policy_type = f"http:{info.get('scheduler', 'unknown')}"

        # 서버가 알아서 리사이즈하므로 shape는 비워둔다 (node가 리사이즈를 건너뜀).
        self.image_features: Dict[str, tuple] = {FRONT_KEY: (), WRIST_KEY: ()}

        if reset_on_init:
            self.reset()

    def reset(self) -> None:
        requests.post(f"{self.server_url}/reset", timeout=self.timeout).raise_for_status()

    def select_action(self, observation: Dict[str, np.ndarray]) -> np.ndarray:
        front = observation.get(FRONT_KEY)
        wrist = observation.get(WRIST_KEY)
        state = observation.get(STATE_KEY)
        if front is None or wrist is None:
            raise ValueError(f"카메라 관측이 없음: {FRONT_KEY} / {WRIST_KEY}")
        if state is None:
            raise ValueError(f"상태 관측이 없음: {STATE_KEY}")

        payload = {
            "front_rgb": _encode_png(front),
            "wrist_rgb": _encode_png(wrist),
            "state": np.asarray(state, dtype=np.float32).reshape(-1).tolist(),
        }
        r = requests.post(f"{self.server_url}/predict", json=payload, timeout=self.timeout)
        r.raise_for_status()
        return np.asarray(r.json()["action"], dtype=np.float32).reshape(-1)
```

`requests`, `pillow` 의존성을 ROS 환경에 설치:
```bash
pip install requests pillow
```
그리고 `lerobot_system/package.xml`에 추가(선택):
```xml
<exec_depend>python3-requests</exec_depend>
```

---

### Step 3 — node.py에서 HTTP 러너로 분기

`lerobot_system/lerobot_system/node.py`를 **최소 수정**한다.

**(a) import 추가** (파일 상단 import 블록):
```python
from lerobot_system.http_policy_runner import HttpPolicyRunner
```

**(b) 파라미터 선언 추가** (다른 `declare_parameter` 들 옆, 예: line 53 근처):
```python
self.declare_parameter("use_http", False)
self.declare_parameter("policy_server_url", "http://localhost:8000")
self.declare_parameter("http_timeout_sec", 5.0)
```

**(c) 러너 생성부 교체** (현재 `self.runner = LeRobotPolicyRunner(runner_cfg)` 부분, line 105~115):
```python
if bool(self.get_parameter("use_http").value):
    self.runner = HttpPolicyRunner(
        server_url=self.get_parameter("policy_server_url").value,
        timeout_sec=float(self.get_parameter("http_timeout_sec").value),
    )
else:
    runner_cfg = PolicyRunnerConfig(
        policy_path=self.get_parameter("policy_path").value,
        device=self.get_parameter("device").value,
        task=self.get_parameter("task").value,
        robot_type=self.get_parameter("robot_type").value,
        local_files_only=bool(self.get_parameter("local_files_only").value),
        use_amp=bool(self.get_parameter("use_amp").value),
        mock_policy=bool(self.get_parameter("mock_policy").value),
        mock_action_size=int(self.get_parameter("mock_action_size").value),
    )
    self.runner = LeRobotPolicyRunner(runner_cfg)

self.required_state_dim = self.runner.state_dim
self.expected_action_dim = self.runner.action_dim
self.image_shapes = self.runner.image_features
```

> 나머지(상태 조립 `_build_observation`, 액션 분할 `_slice_action`,
> arm/hand 발행 `_publish_robot_action`/`_publish_gripper_action`, 단위 변환,
> `max_joint_delta` 클램프, `enable_output` 게이트)는 **그대로 둔다.**

---

## 4. Launch / 파라미터 설정

`lerobot_system/launch/lerobot_system.launch.py` 의 Node 파라미터를 아래로 설정
(또는 별도 launch 작성):

```python
Node(
    package="lerobot_system",
    executable="lerobot_system_left",   # left_main 진입점
    name="lerobot_system_left",
    output="screen",
    parameters=[{
        # --- 정책 백엔드: HTTP ---
        "use_http": True,
        "policy_server_url": "http://localhost:8000",  # 다른 PC면 서버 IP
        "http_timeout_sec": 5.0,

        "side": "left",
        "fps": 30.0,

        # --- 상태 26차원: robot_joint(6) + gripper_joint(20) ---
        "state_fields": ["robot_joint", "gripper_joint"],
        "state_padding_mode": "pad_or_truncate",  # 안전망 (정상이면 26 그대로)

        # --- 카메라 (Step 1에서 만든 토픽) ---
        "camera_topics": [
            "/system_left/camera/front/rgb",
            "/system_left/camera/wrist/rgb",
        ],
        "camera_keys": [
            "observation.images.front_rgb",
            "observation.images.wrist_rgb",
        ],
        "camera_timeout_sec": 1.0,

        # --- action 분할: arm[0:6], hand[6:26] ---
        "robot_action_start": 0,  "robot_action_size": 6,
        "gripper_action_start": 6, "gripper_action_size": 20,

        # --- arm 출력: 모델은 rad, hdr_stream은 deg 구독 → 변환 ---
        "action_output_unit": "rad",
        "robot_topic_unit": "deg",
        "robot_action_topic": "/robot/joint_target_deg",  # hdr_stream 구독 토픽
        "max_joint_delta": 0.0,   # 처음엔 0.05(rad)처럼 작게 걸어 안전 테스트 권장

        # --- hand 출력: DG5F pospid reference (rad) ---
        "enable_gripper_output": True,
        "gripper_action_topic": "/dg5f_left/lj_dg_pospid/reference",
        "gripper_command_type": "multi_dof_command",

        # --- 안전 게이트: 검증 끝나기 전엔 False로 추론만 돌려보기 ---
        "enable_output": False,
    }],
)
```

> `enable_output: False`면 추론과 `/lerobot/left/raw_action` 발행은 되지만
> 실제 로봇/그리퍼로는 안 나간다. 액션 값이 합리적인지 확인 후 `True`로.

---

## 5. 실행 순서

```bash
# 0) 정책 서버 (별도 터미널 / 별도 PC 가능)
cd ~/Tesollo/visuomotor-policy-inference && docker compose up --build

# 1) ROS 워크스페이스 빌드
cd ~/Tesollo/1.Project/1.hyundae/seoul_uiwang/system_Teleop
colcon build --packages-select teleop_vision lerobot_system
source install/setup.bash

# 2) 하드웨어 스택 기동 (각각 별도 터미널)
ros2 launch hdr_stream ...            # HDR35 (실로봇이면 simulation:=false)
ros2 launch dg5f_driver ...           # DG5F 그리퍼 + pospid 컨트롤러
ros2 run net_ft_driver ...            # F/T (state에 필요 없으면 생략 가능)
ros2 run teleop_vision vision_node_left   # 카메라 (Step1 수정본)
ros2 run system_left system_left          # frame_aligned_state 발행

# 3) 중간 폴리시 노드 (enable_output:=false 로 먼저)
ros2 launch lerobot_system lerobot_system.launch.py
```

---

## 6. 검증 & 디버깅

```bash
# 서버 단독 스모크 테스트 (합성 프레임)
python examples/client_example.py --url http://localhost:8000

# 추론 출력 확인 (로봇으로 안 나감, enable_output=false 상태)
ros2 topic echo /lerobot/left/raw_action      # 26개 값
ros2 topic hz   /lerobot/left/raw_action      # ~30Hz 나오는지

# state가 26차원으로 잘 조립되는지 (노드 로그의 state_dim 경고 확인)
#  -> "State dim mismatch" 경고가 뜨면 state_fields가 26을 안 만든 것

# 카메라 도착 확인 (안 오면 _run_once가 _images_ready에서 멈춤)
ros2 topic hz /system_left/camera/front/rgb
ros2 topic hz /system_left/camera/wrist/rgb
```

문제 진단 체크리스트:
- 추론이 안 돈다 → 카메라 두 토픽이 `camera_timeout_sec`(1초) 안에 들어오는지, `frame_aligned_state`가 발행되는지.
- `state must have 26 dims` 류 400 에러 → `state_fields`가 `["robot_joint","gripper_joint"]`인지.
- arm이 안 움직인다 → `robot_action_topic`이 hdr_stream 구독(`/robot/joint_target_deg`)과 일치하는지, `enable_output:=true`인지, 실로봇이면 hdr_stream `simulation:=false`인지.
- 30Hz 미달 → 서버가 CPU면 느림. `VPI_DEVICE=cuda`로 GPU 서빙. 네트워크 PC 분리 시 지연 확인.

---

## 7. 주의사항

1. **left 전용 모델**: 현재 서버 모델은 왼팔용. 오른팔은 별도 모델을 띄우고
   포트를 분리(예: 8001)해서 `lerobot_system_right`가 그쪽을 보게 한다
   (서버는 단일 로봇 큐라 로봇마다 컨테이너 1개).
2. **에피소드 리셋**: 정책은 내부 관측/액션 큐(`n_obs_steps=2`, `n_action_steps=8`)를
   가진다. 새 작업 시작 때 `POST /reset`이 필요하다. 위 러너는 노드 시작 시 1회 reset한다.
   작업 중 수동 리셋이 필요하면 노드에 ROS 서비스(`/lerobot/left/reset`)를 추가해
   `HttpPolicyRunner.reset()`을 호출하게 확장하면 된다.
3. **단위**: 학습 데이터가 라디안 기준이라 모델 출력도 라디안으로 가정한다.
   arm은 rad→deg 변환(설정 완료), hand pospid는 rad 그대로 받는다.
   실제 부호/스케일은 **첫 구동 때 `max_joint_delta`를 작게** 걸고 천천히 검증할 것.
4. **안전**: 첫 통합 테스트는 반드시 `enable_output:=false` → raw_action 값 검토 →
   `max_joint_delta` 작게(예: 0.05 rad) → 실로봇 순으로. 비상정지 항상 확보.
5. **카메라 좌/우 매칭**: `front_rgb`=씬(Zivid), `wrist_rgb`=손목(RealSense)을
   바꿔 넣으면 정책이 엉뚱하게 동작한다. 토픽-키 매핑을 꼭 확인.

---

## 8. 변경 파일 요약

| 파일 | 변경 |
|---|---|
| `Vision_/teleop_vision/vision_node_left.py` | `sensor_msgs/Image` 퍼블리셔 2개 추가 (front/wrist) |
| `System_/lerobot_system/lerobot_system/http_policy_runner.py` | **신규** — HTTP 러너 |
| `System_/lerobot_system/lerobot_system/node.py` | `use_http`/`policy_server_url` 파라미터 + 러너 분기 (3곳) |
| `System_/lerobot_system/launch/lerobot_system.launch.py` | 파라미터 설정 (Step 4) |
| `System_/lerobot_system/package.xml` | `python3-requests` exec_depend (선택) |
```
