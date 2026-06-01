# 프로젝트 개요 — Visuomotor Policy 로봇 실행 시스템

> **최종 목표**: 로봇(HDR35 팔 + DG5F 손)을 **init state(home)로 복귀시킨 뒤, 그 상태에서
> 학습된 정책으로 한 에피소드를 실행**하고, 끝나면 다시 init state로 돌아와 **반복**한다.
> 입력은 카메라 2대(Zivid/RealSense) + 로봇 상태, 정책 출력은 로봇 action.

이 문서는 전체 그림 · 문서 색인 · 진행 상태 · 남은 작업을 한 곳에 모은 인덱스다.

---

## 1. 전체 아키텍처

```
┌──────────────────────────────────────────────────────────────────────────┐
│  Frontend (웹)  HOME/START/STOP/CLEAR/E-STOP + 상태표시   [frontend/]      │
│       │  ws (rosbridge)                                                    │
│  ┌────▼─────────────────────────────────────────────────────────────┐    │
│  │  Layer 3: Episode Manager — 상태머신·홈복귀·게이팅·안전  [설계완료]│    │
│  │     IDLE→HOMING→READY→RUNNING→STOPPING→(HOMING)  / FAULT           │    │
│  └────┬──────────────────────────────────────────────────────────────┘    │
│       │ run gate · reset · home 명령 · abort                               │
│  ┌────▼──────────────────────────────────────────────────────────────┐    │
│  │  Layer 2: Policy Control — 추론→action→arm/hand  [구현완료]        │    │
│  │     ros2_robot_client (vpi_robot_client)                          │    │
│  └────┬───────────────────────────────────┬──────────────────────────┘    │
│       │ /predict (HTTP, JPEG)              ▲ state(26)+cameras             │
│  ┌────▼───────────────────┐     ┌──────────┴──────────────────────────┐   │
│  │ Policy Server (GPU)    │     │  Layer 1: 입력/구동                  │   │
│  │ visuomotor-policy-inf  │     │  카메라2 · 관절/그리퍼 state · FT ·  │   │
│  │ [구현·GPU검증 완료]    │     │  HDR35 드라이버 · DG5F pospid        │   │
│  └────────────────────────┘     └───────────────────────────────────────┘  │
└──────────────────────────────────────────────────────────────────────────┘
```

**데이터 규격 (확정)**
- 정책 입력: 카메라 `front_rgb`(Zivid)·`wrist_rgb`(RealSense) + **state(26)=arm관절6 + 그리퍼관절20 (rad)**
- 정책 출력: **action(26)=arm6 + hand20, 절대 목표 관절(rad), 30Hz**
- arm → rad→deg → `/robot/joint_target_deg` → HDR35 / hand → `MultiDOFCommand(rad)` → `/dg5f_left/lj_dg_pospid/reference` → DG5F
- ⚠️ FT/촉각 센서는 **정책 입력이 아니라** 안전·로깅용

---

## 2. 문서 색인

| 문서 | 내용 |
|---|---|
| **[README.md](./README.md)** | 서버 개요 + Performance(RTX 3060) |
| **[BENCHMARK.md](./BENCHMARK.md)** | 레이턴시·밴드위드 측정 결과 + 권장안 |
| **[INTEGRATION_HDR35.md](./INTEGRATION_HDR35.md)** | 입력(카메라/state) 연동 + lerobot_system 배선 |
| **[EXECUTION_PLAN.md](./EXECUTION_PLAN.md)** | 모델 output → 로봇 실행(안전·소프트스타트·단계 브링업) |
| **[EPISODE_SYSTEM.md](./EPISODE_SYSTEM.md)** | 에피소드 실행 시스템 구조(상태머신·홈복귀·인터페이스 계약) |
| **[ros2_robot_client/README.md](./ros2_robot_client/README.md)** | 정책 제어 ROS2 노드 빌드/실행 |
| **[frontend/README.md](./frontend/README.md)** | 웹 콘솔 실행 |

---

## 3. 성능 요약 (RTX 3060)

- **모델 추론 ~40Hz** (디퓨전 forward 1회 ≈24ms), 청크=8이라 실효 제어 ~128Hz. VRAM ~1.4GB.
- **30Hz에 충분** — 병목은 추론이 아니라 HTTP 이미지 전송(JPEG/저해상도로 해결).
- 자세한 수치는 BENCHMARK.md.

---

## 4. 진행 상태

**완료 ✅**
- [x] conda 환경 `vpi` (Python 3.11) + lerobot 포크 + torch CUDA
- [x] 모델 다운로드 (left/right flowmatch, 각 1.1GB)
- [x] Policy Server GPU 추론 검증 (action[26] 정상 출력)
- [x] 레이턴시·밴드위드 벤치마크 (`scripts/benchmark.py`, BENCHMARK.md)
- [x] Policy Control 노드 (`vpi_robot_client`) — 추론→arm/hand, 소프트스타트, limit clamp, `~/reset`
- [x] Episode 실행 시스템 **설계** (EPISODE_SYSTEM.md)
- [x] 웹 프론트엔드 (`frontend/index.html`)

**남은 작업 (의존 순서) ⏳**
- [ ] (친구) 카메라 2토픽 발행 — `/system_left/camera/front|wrist/rgb`
- [ ] (친구) state(26) 발행 — FrameAlignedState 재사용 or joint_states
- [ ] (친구) FT/촉각 센서 토픽 (안전·로깅)
- [ ] (우리) Policy Control에 `/vpi/set_enable` 런타임 게이트 추가
- [ ] (우리) **Episode Manager 노드** 구현 — 상태머신+homing+watchdog+서비스/status
- [ ] (옵션) cmd_mux 하드 중재, 에피소드 레코더, success 자동분류기

---

## 5. 시작 전 결정할 것

1. **home 자세값** (arm6+hand20): **학습 데이터 에피소드 시작상태에서 추출 권장** (분포 일치). 데이터셋에서 뽑아줄 수 있음.
2. **종료 조건**: 초기엔 `timeout + manual + safety`, success 자동판정은 추후.
3. **반복 모드**: 수동(매번 START) vs 자동(episodes 카운트 + auto_reset/auto_start).
4. **명령 중재**: 게이트 기반(기본) vs cmd_mux(하드).
5. **state 소스**: FrameAlignedState vs joint_states.

---

## 6. 기동 순서 (완성 시)

```bash
# 1) 정책 서버 (GPU)
conda activate vpi && VPI_DEVICE=cuda uvicorn app.server:app --host 0.0.0.0 --port 8000

# 2) 로봇 드라이버 + 카메라 + state (Layer 1)
ros2 launch hdr_stream ...          # HDR35 (검증 전엔 simulation:=true)
ros2 launch dg5f_driver ...         # DG5F + pospid
ros2 run teleop_vision vision_node_left   # 카메라 2토픽 (친구 작업 후)

# 3) 정책 제어 (출력 OFF로 먼저)
ros2 launch vpi_robot_client policy_control.launch.py enable_output:=false

# 4) Episode Manager (구현 후)
ros2 run episode_manager episode_manager

# 5) 프론트엔드
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
cd frontend && python -m http.server 8080      # http://localhost:8080
```

브링업 단계: **추론만(차단) → 시뮬 → 실로봇 보수값 → 정상값** (EXECUTION_PLAN.md §3).

---

## 7. 디렉토리

```
visuomotor-policy-inference/
├── app/                      # FastAPI 추론 서버 (server.py, policy_runner.py, schemas.py)
├── scripts/benchmark.py      # 레이턴시·밴드위드 벤치마크
├── ros2_robot_client/        # 정책 제어 ROS2 노드 (vpi_robot_client) [구현완료]
├── frontend/                 # 웹 에피소드 콘솔 [구현완료]
├── PROJECT_OVERVIEW.md       # (이 문서)
├── EPISODE_SYSTEM.md  EXECUTION_PLAN.md  INTEGRATION_HDR35.md  BENCHMARK.md
├── README.md  Dockerfile  docker-compose.yml  requirements.txt
└── (Episode Manager 노드는 추후 추가)
```
