# 모델 output → 로봇 실행(Execution) 구현 계획

> 범위: 정책 서버가 돌려준 `action[26]`을 **HDR35 팔 + DG5F 손에서 안전하게 실행**하는 부분.
> 입력(카메라/state) 수집은 `INTEGRATION_HDR35.md` 참고. 이 문서는 **출력 실행단**에 집중.

---

## 0. 먼저 — action이 뭔지 (확정 사실)

모델 카드 + `inference_example.py` + `config.json`으로 확인한 출력 규격:

| 항목 | 값 | 의미 |
|---|---|---|
| `action` shape | `[26]` | `arm[0:6]` + `hand[6:26]` |
| 값의 종류 | **절대 목표 관절값(absolute target joint)** | 델타 아님. "다음에 가야 할 관절 위치" |
| 단위 | **라디안** (학습 데이터가 rad) | arm 6 = HDR35 관절 rad, hand 20 = DG5F 관절 rad |
| 정규화 | MIN_MAX → 서버 `postprocess`가 **역정규화 완료** | 서버가 주는 값은 이미 실제 rad |
| 시간축 | 30 Hz, `n_action_steps=8` chunk를 내부 큐로 관리 | `/predict` 1회 = action 1개 |

→ **핵심 함의**: action은 절대 위치다. 그래서 **첫 틱에서 현재 자세와 목표가 멀면 로봇이 튄다.**
실행 설계의 절반은 이 "절대 목표를 안전하게 추종"시키는 안전장치다.

---

## 1. 실행 경로 (두 갈래)

`lerobot_system` 노드가 `action`을 잘라서 두 토픽으로 발행하고, 그 뒤는 기존 실행단이 받는다.

### 1-A. 팔 (arm 6) 경로

```
action[0:6] (rad, 절대)
  └─ lerobot_system: rad→deg 변환 + max_joint_delta 클램프
       └─ publish JointState(deg) → /robot/joint_target_deg
            └─ hdr_stream._on_target_joint(): latest_target_deg 갱신
                 └─ send_worker() @200Hz: joint_traject_insert_point 로 스트리밍
                      (interval, time_from_start 누적, look_ahead_time=0.2)
                       └─ HDR35 컨트롤러가 룩어헤드 보간하며 추종
```

**기존 실행단의 동작 (hdr_stream.py 확인 결과):**
- `send_worker`는 **200Hz**(`send_dt_sec=0.005`)로 돌며 `latest_target_deg`를 계속 로봇에 스트리밍.
- 정책은 **30Hz**로 `latest_target_deg`를 갱신 → 한 정책 틱 동안 같은 목표를 ~6~7번 insert.
  로봇이 `look_ahead_time=0.2`로 보간하므로 30Hz step 입력이 부드럽게 추종됨.
- **이미 들어있는 안전장치**:
  - 관절 속도 가드: 실모드에서 `|현재 - 목표| > 25°`면 즉시 `api.stop()` + `overflow_workspace` 래치 → 전송 정지 (hdr_stream.py:137-146)
  - 카테시안 작업영역 한계: left/right별 x/y/z 범위 초과 시 stop (hdr_stream.py:157-166)
- **단위**: hdr_stream은 deg를 받아 서버가 내부적으로 rad 변환. 그래서 정책 노드는 `rad→deg`로 보내야 함.

### 1-B. 손 (hand 20) 경로

```
action[6:26] (rad, 절대)
  └─ lerobot_system: (현재 클램프 없음) publish MultiDOFCommand(rad)
       → /dg5f_left/lj_dg_pospid/reference
            └─ ros2_control PidController "lj_dg_pospid"
               (reference_and_state_interfaces: ["position"], 20개 lj_dg_*_* 관절)
                 └─ dg5f_operator_driver: TCP로 그리퍼 하드웨어에 위치 명령
```

**확인 결과 (dg5f config):**
- 컨트롤러는 `pid_controller/PidController`, reference 인터페이스 = **position(rad)**.
- `MultiDOFCommand{ dof_names=[lj_dg_1_1..lj_dg_5_4], values=[rad 20개] }` 형태로 받음 → lerobot_system이 이미 이 포맷으로 발행.
- ⚠️ **손 경로엔 속도 클램프가 없다** → 첫 틱 점프 위험 그대로. (아래 2-B에서 보강)

---

## 2. 반드시 처리해야 할 실행 안전 이슈 + 해결책

### 2-A. 첫 틱 점프 (절대 목표 → 급가속) — 팔

**문제**: 정책 첫 action이 현재 자세에서 멀면 한 번에 큰 목표가 들어가 로봇이 튀거나
25° 속도 가드에 걸려 stop.

**해결**: `lerobot_system`의 `max_joint_delta`(rad)를 **켠다**. 코드상
`target = current + clip(action - current, -Δ, +Δ)` 라서, 매 30Hz 틱마다 **현재 측정 관절** 기준
Δ만큼만 이동 → 자동 소프트스타트 + 속도 제한.

```
max_joint_delta = 0.02   # rad/tick @30Hz ≈ 0.6 rad/s ≈ 34°/s  (브링업용, 보수적)
# 검증 후 0.05(≈1.5rad/s)까지 올려도 됨. 25°/step 하드가드보다 훨씬 작게 유지.
```

### 2-B. 첫 틱 점프 — 손 (보강 필요)

현재 `_publish_gripper_action`은 raw로 쏜다. 손가락도 절대 목표라 급격히 닫힐 수 있음.
→ **손 전용 rate limiter 추가** (node.py 소폭 수정):

```python
# __init__
self.declare_parameter("max_gripper_delta", 0.0)   # rad/tick, 0=off
self.max_gripper_delta = float(self.get_parameter("max_gripper_delta").value)
self._last_gripper_cmd = None   # 직전 명령(rad) 보관

# _publish_gripper_action 시작부에 추가
if self.max_gripper_delta > 0.0:
    base = self._last_gripper_cmd
    if base is None:
        # 첫 틱: 현재 손 관절(state.gripper_joint)에서 출발해야 점프 없음
        base = np.asarray(state.gripper_joint, dtype=np.float64)[:len(gripper_action)]
    delta = np.clip(gripper_action - base, -self.max_gripper_delta, self.max_gripper_delta)
    gripper_action = base + delta
self._last_gripper_cmd = np.asarray(gripper_action, dtype=np.float64)
```
> 이를 위해 `_publish_gripper_action`에 `state`를 인자로 넘기도록 호출부를 한 줄 수정.

### 2-C. 관절 한계 클램프 (limit clamp)

정책이 학습분포 밖 값을 낼 수 있음 → **URDF joint limit으로 클램프** 후 전송.
- 팔: HDR35 모델 한계 (hdr_description URDF, 또는 hdr_stream에 상수로).
- 손: `dg_description` URDF의 lj_dg_*_* limit (또는 DG5F 스펙 표).

권장: 정책 노드에서 `action`을 받은 직후 `np.clip(action, q_min, q_max)` 1줄 적용
(arm/hand 각각의 limit 벡터를 파라미터로). 하드가드(25°·작업영역)는 최후의 방어선이고,
clip은 그 전에 정상 범위를 보장.

### 2-D. 레이트/지연 예산 (30Hz)

한 틱 33ms 안에: 카메라 2장 인코딩 + HTTP 왕복 + 추론(FlowMatch `num_inference_steps=1`) + 발행.
- 서버 **GPU 필수** (`VPI_DEVICE=cuda`). CPU면 30Hz 불가.
- 서버를 로봇 PC와 분리하면 네트워크 지연 추가 → 같은 PC 또는 저지연 LAN.
- `n_action_steps=8` 덕분에 추론은 8틱에 1번 무거움(replanning), 나머지는 큐에서 pop → 평균 부하는 낮음.
- 측정: `ros2 topic hz /lerobot/left/raw_action` 가 ~30Hz 유지되는지 확인.

### 2-E. 에피소드 리셋

정책은 관측/액션 큐(`n_obs_steps=2`, `n_action_steps=8`)를 가짐. 작업 시작마다 `POST /reset` 필요.
→ 노드에 ROS 서비스 `/lerobot/left/reset` 추가해서 `HttpPolicyRunner.reset()` 호출하게.
시작 시 1회는 러너 `__init__`에서 자동 reset(이미 반영).

### 2-F. 출력 게이트 / 비상정지

- `enable_output=false`로 먼저 **추론만** 돌려 `raw_action` 관찰 (로봇 안 움직임).
- hdr_stream의 `overflow_workspace` 래치가 걸리면 재시작 전까지 전송 정지 → 정상.
- 하드웨어 E-stop 물리 버튼 항상 확보.

---

## 3. 단계별 브링업 절차 (이 순서를 지킬 것)

**Stage 0 — 서버 단독**
```bash
conda activate vpi   # 이미 모델은 ~/.cache/huggingface 에 받아둠
python ~/.cache/huggingface/hub/models--Ngseo--hyundai-uiwang-left-flowmatch/snapshots/*/inference_example.py --device cuda
# 또는 서버: cd visuomotor-policy-inference && VPI_DEVICE=cuda docker compose up --build
curl localhost:8000/info   # action_dim=26 확인
```

**Stage 1 — 추론만 (로봇 차단)**
- `enable_output:=false`로 정책 노드 실행.
- 확인: `ros2 topic echo /lerobot/left/raw_action` → 26개 값이 **합리적 범위(rad)**인지,
  `ros2 topic hz` → 30Hz인지, state가 26차원으로 조립되는지(노드 로그 경고 無).

**Stage 2 — 시뮬레이션 추종**
- hdr_stream `simulation:=true` (실로봇 미전송, 목표 로그만), dg5f는 Gazebo(`Teleop`의 dg5f_gz) 또는 RViz.
- `max_joint_delta:=0.02`, `max_gripper_delta:=0.02`로 소프트스타트 확인. 목표가 현재에서 부드럽게 출발하는지.

**Stage 3 — 실로봇, 보수적**
- hdr_stream `simulation:=false`, `enable_output:=true`, `enable_gripper_output:=true`.
- `max_joint_delta:=0.02`, `max_gripper_delta:=0.02` 유지. 손은 사람이 비상정지 대기.
- 작은 동작부터: 손을 로봇 작업영역 안전 위치에 두고 짧게.

**Stage 4 — 정상 운용**
- 검증되면 `max_joint_delta`/`max_gripper_delta`를 0.04~0.05로 상향(부드러움↑, 추종성↑).
- 25°/step·작업영역 하드가드는 끝까지 유지.

---

## 4. lerobot_system 파라미터 (실행단 관점 최종값)

```python
parameters=[{
  # 정책 출력 분할
  "robot_action_start": 0,  "robot_action_size": 6,    # arm
  "gripper_action_start": 6, "gripper_action_size": 20, # hand

  # 팔 실행
  "action_output_unit": "rad", "robot_topic_unit": "deg",  # 모델 rad → hdr_stream deg
  "robot_action_topic": "/robot/joint_target_deg",
  "max_joint_delta": 0.02,        # ★ 소프트스타트 + 속도 제한 (브링업)

  # 손 실행
  "enable_gripper_output": True,
  "gripper_action_topic": "/dg5f_left/lj_dg_pospid/reference",
  "gripper_command_type": "multi_dof_command",
  "max_gripper_delta": 0.02,      # ★ 2-B에서 추가하는 파라미터

  # 게이트
  "enable_output": False,         # Stage 3에서 True
  "fps": 30.0,
}]
```

---

## 5. 손봐야 할 코드 (실행단 한정)

| 파일 | 변경 | 이유 |
|---|---|---|
| `lerobot_system/node.py` | `max_gripper_delta` 파라미터 + 손 rate limiter (2-B), 호출부에 `state` 전달 | 손 첫 틱 점프 방지 |
| `lerobot_system/node.py` | (선택) action limit clamp 파라미터(`arm/hand min·max`) + 발행 전 clip (2-C) | 분포 밖 출력 방어 |
| `lerobot_system/node.py` | (선택) `/lerobot/{side}/reset` 서비스 → `runner.reset()` (2-E) | 에피소드 경계 |
| launch 파일 | §4 파라미터 | 실행 설정 |

> 팔 경로는 hdr_stream의 기존 스트리밍/안전장치를 **그대로 재사용**하므로 추가 코드 없음.
> `max_joint_delta`는 이미 node.py에 구현돼 있어 **값만 설정**하면 됨.

---

## 6. 한 줄 요약

- action = **절대 목표 관절(rad)**. 팔은 `rad→deg`로 `/robot/joint_target_deg`에 쏘면
  hdr_stream이 200Hz 스트리밍+룩어헤드로 추종(속도/작업영역 하드가드 내장).
  손은 `MultiDOFCommand(rad)`로 pospid 컨트롤러에 쏘면 됨.
- 실행 설계의 핵심은 **절대 목표의 첫 틱 점프를 막는 것** → 팔 `max_joint_delta`(기구현),
  손 `max_gripper_delta`(추가)로 소프트스타트. limit clamp는 보조 방어.
- `enable_output=false → 시뮬 → 실로봇 보수값 → 정상값` 순서로 단계 브링업.
