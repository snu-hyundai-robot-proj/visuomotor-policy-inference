# 에피소드 실행 시스템 설계 (Episode Execution System)

> 최종 목표: **로봇을 init state(home)로 복귀시킨 뒤, 그 상태에서 정책으로 한 에피소드를
> 실행하고, 끝나면 다시 init state로 돌아와 다음 에피소드를 반복**할 수 있는 시스템.
>
> 이 문서는 그 **오케스트레이션 구조**를 정의한다. 정책 추론→로봇 실행 자체는
> [`ros2_robot_client`](./ros2_robot_client) (= `EXECUTION_PLAN.md`)가 이미 담당하고,
> 여기서는 그 위에 **에피소드 생명주기(상태머신) + 홈 복귀 + 안전 게이팅**을 얹는다.

---

## 1. 핵심 아이디어 — 3개의 레이어

```
┌──────────────────────────────────────────────────────────────────────┐
│  Layer 3: Episode Manager (상태머신·홈복귀·게이팅·안전)  ← 새로 설계   │
│     IDLE → HOMING → READY → RUNNING → STOPPING → (다시 HOMING)         │
└───────────────┬──────────────────────────────────────────────────────┘
                │ run gate(ON/OFF) · policy reset · home 명령 · 안전 abort
┌───────────────▼──────────────────────────────────────────────────────┐
│  Layer 2: Policy Control (정책 추론 → action → arm/hand 명령)  ← 완료   │
│     vpi_robot_client.policy_control_node                               │
└───────────────┬──────────────────────────────────────────────────────┘
                │ /predict (HTTP)            ▲ state(26) + cameras
┌───────────────▼─────────────┐   ┌──────────┴───────────────────────────┐
│  Policy Server (GPU)         │   │  Layer 1: 입력/구동 (드라이버·센서)   │
│  visuomotor-policy-inference │   │  카메라2 · 관절/그리퍼 상태 · FT ·    │
└──────────────────────────────┘   │  HDR35 드라이버 · DG5F pospid         │
                                    └───────────────────────────────────────┘
```

핵심: **"홈 복귀"와 "정책 실행"은 둘 다 같은 로봇 명령 토픽**(`/robot/joint_target_deg`,
`/dg5f_left/lj_dg_pospid/reference`)**에 쓴다.** 동시에 쓰면 안 되므로, 상태머신이
**한 시점에 한 source만 쓰도록 게이팅**한다 (아래 §5 명령 중재).

---

## 2. 컴포넌트 & 역할

| # | 컴포넌트 | 역할 | 누가 | 상태 |
|---|---|---|---|---|
| 1 | **Camera nodes** (front/wrist) | Zivid·RealSense RGB를 ROS Image로 발행 | 친구 | TODO |
| 2 | **State aggregator** | arm 관절(6)+그리퍼 관절(20) → state(26) 정렬 발행 | 친구/재사용 | TODO/일부 |
| 3 | **Sensor nodes** | FT(6)·tactile(30) — **안전·로깅용** (이 모델 입력 아님!) | 친구 | TODO |
| 4 | **Robot drivers** | HDR35(`hdr_stream`)·DG5F(`dg5f` pospid) | 기존 | 완료 |
| 5 | **Policy control** | state+cameras→/predict→arm/hand 명령 (게이트형) | 우리 | 완료(게이트 런타임화 필요) |
| 6 | **Episode Manager** | 상태머신·홈복귀·게이팅·안전·서비스 API | 우리 | **설계(이 문서)** |
| 7 | **Policy Server** | GPU 추론 (HTTP) | 완료 | 완료 |

> ⚠️ **중요**: 이 모델의 정책 입력은 **arm 관절 6 + 그리퍼 관절 20 = state(26) + 카메라 2장**뿐이다.
> FT/촉각 센서는 **정책 입력이 아니라** 안전(과부하 abort)·데이터 로깅 용도다. 친구가 센서를 만들 때
> "정책에 꼭 필요한 것"과 "안전/로깅용"을 구분해야 한다.

---

## 3. 에피소드 상태머신

```
            ┌─────────┐
            │  BOOT   │  노드 기동, 입력 헬스 대기
            └────┬────┘
        all healthy │
            ┌────▼────┐   /episode/home
   ┌───────►│ HOMING  │◄────────────────┐   home 목표로 rate-limited 복귀
   │        └────┬────┘                 │   (정책 게이트 OFF, 매니저가 명령)
   │     at-home & settled │            │
   │        ┌────▼────┐                 │
   │        │  READY  │  홈 정착 + 헬스 OK + policy reset 완료, 시작 대기
   │        └────┬────┘                 │
   │   /episode/start │                 │
   │        ┌────▼────┐                 │
   │        │ RUNNING │  정책 게이트 ON, 정책이 로봇 구동              │
   │        └────┬────┘                 │
   │  종료트리거(아래)│                 │
   │        ┌────▼────┐                 │
   └────────┤STOPPING │  게이트 OFF, 현재자세 hold → (auto_reset이면) HOMING
   auto_reset└────┬────┘
                  │ 안전위반(어느 상태든)
             ┌────▼────┐
             │  FAULT  │  래치됨. 게이트 OFF·hold. /episode/clear_fault 필요
             └─────────┘
```

**전이(transition)와 트리거**
| from → to | 트리거 | 가드(조건) |
|---|---|---|
| BOOT → HOMING | 자동 | 모든 입력 신선(카메라·state) |
| any → HOMING | `/episode/home` | not FAULT |
| HOMING → READY | 자동 | home 허용오차 내 N틱 정착 + 헬스OK + policy reset 성공 |
| READY → RUNNING | `/episode/start` | READY 상태 + 헬스OK |
| RUNNING → STOPPING | 종료트리거(§10) | — |
| STOPPING → HOMING | 자동 | `auto_reset=true` (아니면 STOPPING→READY는 수동 home) |
| any → FAULT | 안전위반(§8) | — |
| FAULT → HOMING | `/episode/clear_fault` | 위반 해소 |

> `episodes_remaining` 카운터를 두면 STOPPING→HOMING→READY→(auto_start면)RUNNING 으로
> **N회 자동 반복**(배치 평가) 가능. 수동 모드면 매 에피소드 `/episode/start`.

---

## 4. 에피소드 1회 타임라인 (시퀀스)

```
[READY] ──/episode/start──▶ Manager:
   1. 헬스 재확인 (카메라2 신선, state 신선, 서버 reachable, FT 정상, not at-limit)
   2. POST /reset  (정책 큐 초기화 — 새 에피소드)         [policy_control ~/reset]
   3. 정책 게이트 ON  (set_enable true)                   [policy_control /set_enable]
   4. t0 = now,  watchdog 가동
[RUNNING] ── 매 30Hz (정책 control thread가 자체 구동) ──▶
   - state(26)+카메라 → /predict → action(26)
   - arm/hand 소프트스타트·limit clamp → 로봇 명령 발행
   - Manager는 명령을 내지 않음(게이트로 위임), 안전/종료만 감시
종료트리거 발생 (timeout | success | abort | manual):
[STOPPING] ── Manager:
   5. 정책 게이트 OFF  (즉시 로봇 명령 중단)
   6. 현재 자세 hold (마지막 측정 관절을 목표로 잠깐 유지) — 급정지 충격 방지
   7. (옵션) 에피소드 결과/로그 마감
   8. auto_reset이면 → [HOMING]
[HOMING] ── Manager가 단독 명령:
   9. 측정 현재관절 → home 목표로 rate-limited 보간 발행 (게이트 OFF 유지)
   10. home 허용오차 내 정착 → [READY] (필요시 episodes_remaining--, auto_start면 1번으로)
```

---

## 5. 명령 중재 (Arbitration) — 가장 중요한 설계점

`/robot/joint_target_deg` 와 `/dg5f_left/lj_dg_pospid/reference` 에 **쓰는 주체가
상태에 따라 바뀐다**: HOMING/STOPPING = Manager, RUNNING = Policy. 동시에 쓰면 위험.

**채택안 (게이트 기반 상호배제)** — 단순·견고:
- Policy control 노드에 **런타임 enable 게이트** 추가 (`/vpi/set_enable`, `std_srvs/SetBool`).
- Manager가 상태 진입 시 게이트를 토글:
  - RUNNING 진입: 게이트 **ON** (이때만 Policy가 발행), Manager는 발행 안 함.
  - HOMING/STOPPING/READY/FAULT: 게이트 **OFF**, Manager만 발행(홈/hold).
- 전이 순서를 엄격히: **RUNNING→STOPPING은 "게이트 OFF 먼저"**, **READY→RUNNING은
  "Manager 발행 중단 후 게이트 ON"** → 한 시점에 한 source만.

**옵션 (하드 보장)**: 별도 `cmd_mux` 노드를 두어 (Manager 명령 / Policy 명령) 중
활성 source만 드라이버로 통과시키는 먹스를 둘 수 있다. 안전 임계 시스템이면 권장.
일단은 게이트 기반으로 충분.

---

## 6. 인터페이스 계약 (친구가 만들 것 포함)

### 6-A. 카메라 (친구) — **필수**
| 토픽 | 타입 | 비고 |
|---|---|---|
| `/system_left/camera/front/rgb` | `sensor_msgs/Image` (rgb8/bgr8) | Zivid 씬 뷰, ≥30Hz |
| `/system_left/camera/wrist/rgb` | `sensor_msgs/Image` (rgb8/bgr8) | RealSense 손목 뷰, ≥30Hz |

> 해상도 무관(서버가 240×320으로 내부 리사이즈). **front=Zivid, wrist=RealSense 매핑을 절대 바꾸지 말 것.**
> 대역폭 위해 정책 노드는 JPEG로 보냄(친구는 raw Image만 발행하면 됨).

### 6-B. 상태(state) — **필수** (둘 중 하나)
정책 입력 state(26) = `arm 관절(6, rad)` + `gripper 관절(20, rad)`.
- **방식①**: 기존 `system_interface/FrameAlignedState` 재사용 (`robot_joint`+`gripper_joint`). 정렬·rad 보장.
- **방식②**: 두 `sensor_msgs/JointState` 따로 발행 → 정책 노드 `state_source:=joint_states`로 합침.
  - arm: `/system_left/joint_states` (단위 명시, hdr_stream은 deg)
  - hand: `/dg5f_left/joint_states` (rad)

### 6-C. 센서 (친구) — **안전·로깅용** (정책 입력 아님)
| 토픽(예) | 타입 | 용도 |
|---|---|---|
| `/system_left/ft` 또는 net_ft | `geometry_msgs/WrenchStamped` | 과부하 abort, 로깅 |
| `/dg5f_left/tactile` | (커스텀/Float array) | 로깅(향후 입력 확장 대비) |

### 6-D. Episode Manager API (우리) — 서비스/토픽
| 이름 | 타입 | 설명 |
|---|---|---|
| `/episode/home` | `std_srvs/Trigger` | HOMING 시작 |
| `/episode/start` | `std_srvs/Trigger` | READY→RUNNING |
| `/episode/stop` | `std_srvs/Trigger` | 수동 종료 |
| `/episode/clear_fault` | `std_srvs/Trigger` | FAULT 해제 |
| `/episode/success` | `std_msgs/Bool` (sub) | 외부 성공신호(옵션, §10) |
| `/episode/estop` | `std_msgs/Bool` (sub) | 소프트 e-stop |
| `/episode/status` | `std_msgs/String` (JSON) | 현재 state·경과시간·헬스 플래그 (아래 스키마) |

`/episode/status` JSON 스키마 (프론트엔드 `frontend/index.html`가 파싱):
```json
{
  "state": "READY",                 // BOOT|HOMING|READY|RUNNING|STOPPING|FAULT
  "episode_elapsed_s": 0.0,
  "episodes_remaining": 0,
  "policy_enabled": false,
  "fault_reason": "",
  "last_termination": "",           // timeout|manual|success|safety
  "health": {"front_cam": true, "wrist_cam": true, "state": true, "server": true, "ft": true}
}
```
> 커스텀 msg 빌드를 피하려 **JSON String**으로 발행 → 프론트엔드(rosbridge)가 그대로 파싱.

### 6-E. Policy control 노드에 추가할 것 (우리, 소폭)
| 이름 | 타입 | 설명 |
|---|---|---|
| `/vpi/set_enable` | `std_srvs/SetBool` | 런타임 출력 게이트(현재 launch param `enable_output`을 런타임화) |
| `~/reset` | `std_srvs/Trigger` | (이미 있음) 에피소드 시작 시 정책 큐 리셋 |

---

## 7. init state(home) 복귀 설계

- **home 정의**: 파라미터로 고정 — `arm_home`(6, rad 또는 deg) + `hand_home`(20, rad).
  (예: 팔은 작업 시작 자세, 손은 약간 벌린 상태.)
- **복귀 동작**: Manager가 **측정 현재 관절**에서 home까지 **rate-limited 보간**으로
  명령을 발행(틱당 `home_max_delta`). 정책의 소프트스타트와 동일 원리라 급가속 없음.
- **정착 판정**: 모든 관절이 home 허용오차(`home_tol`) 내에 **N틱 연속** → 정착.
  `home_timeout` 초과 시 → FAULT.
- **순서**: 보통 **손 먼저 안전 자세(열기)** → 팔 home → 손 home 자세. 충돌·물체 끼임 방지.
  (시퀀스는 파라미터/스크립트화 가능.)

---

## 8. 안전 / 워치독 (상태 무관, 항상 동작)

RUNNING 중 위반 → 즉시 **STOPPING(또는 FAULT)**. 게이트 OFF + hold.

| 감시 | 조건 | 조치 |
|---|---|---|
| 입력 신선도 | 카메라/state가 `timeout` 내 안 옴 | RUNNING 중이면 abort |
| FT 과부하 | `|F|>F_max` 또는 `|τ|>τ_max` | FAULT |
| 작업영역 | HDR35 `hdr_stream`이 자체 래치(`overflow_workspace`)로 stop | Manager는 상태감지·FAULT |
| 관절속도 | `hdr_stream` 25°/step 하드가드(실모드) | 내장 |
| e-stop | `/episode/estop=true` | FAULT |
| 에피소드 시간 | `t-t0 > max_duration` | 정상 종료(timeout) |

> hdr_stream·dg5f의 **기존 하드가드는 최후 방어선**이고, Manager의 게이트/abort가 1차 방어선.

---

## 9. 종료 조건 (pluggable)

RUNNING을 끝내는 트리거 — 여러 개를 조합:
1. **timeout** (내장): `max_duration` 경과 → 정상 종료.
2. **manual**: `/episode/stop`.
3. **success 신호** (옵션): `/episode/success=true` 구독. 누가 발행? — (a) 운영자 버튼,
   (b) 비전 기반 성공 분류기(별도 노드), (c) FT/상태 기반 룰. **현재는 (a) 수동 권장**, 분류기는 추후.
4. **safety abort**: §8.

> 결정 필요: 초기엔 **timeout + manual** 만으로 시작하고, success 자동판정은 나중에 플러그인.

---

## 10. 기존 시스템 매핑 & 추가 작업

| 기능 | 매핑 | 추가 작업 |
|---|---|---|
| arm 구동 | `hdr_stream` ← `/robot/joint_target_deg` | 없음(재사용) |
| hand 구동 | `dg5f` pospid ← `/dg5f_left/lj_dg_pospid/reference` | 없음 |
| 정책 추론·실행 | `vpi_robot_client` | **`/vpi/set_enable` 런타임 게이트 추가** |
| state(26) | `FrameAlignedState` 또는 joint_states | 친구: 발행 확인 |
| 카메라 | (없음) | 친구: Image 2토픽 발행 (`INTEGRATION_HDR35.md` Step1) |
| 에피소드 생명주기 | (없음) | **Episode Manager 노드 신규** |
| home 복귀 | (없음) | Episode Manager 내부 homing 루틴 |

---

## 11. 빌드 체크리스트

**완료**
- [x] Policy Server (GPU, ~40Hz 추론 검증)
- [x] Policy Control 노드 (`vpi_robot_client`): 추론→arm/hand, 소프트스타트, limit clamp, `~/reset`

**해야 할 것 (의존 순서)**
- [ ] (친구) 카메라 2토픽 발행 — front/wrist
- [ ] (친구) state(26) 발행 경로 확정 — FrameAlignedState 재사용 or joint_states
- [ ] (친구) FT/촉각 센서 토픽 — 안전·로깅용
- [ ] (우리) Policy Control에 `/vpi/set_enable` 런타임 게이트 추가
- [ ] (우리) **Episode Manager 노드** 구현: 상태머신 + homing + 게이팅 + watchdog + 서비스 API
- [ ] (우리) `EpisodeStatus` 메시지 정의(또는 String로 시작)
- [ ] (옵션) `cmd_mux` 하드 중재 노드
- [ ] (옵션) 에피소드 레코더(eval 로깅), success 자동분류기

**시작 전 결정할 것**
1. home 자세 값(arm 6 + hand 20)과 복귀 시퀀스(손 먼저? 팔 먼저?)
2. 종료 조건: 초기엔 timeout+manual 로 갈지 / success 신호를 언제 도입할지
3. 반복 모드: 수동(매번 start) vs 자동(episodes 카운트 + auto_reset/auto_start)
4. 명령 중재: 게이트 기반(기본) vs cmd_mux(하드)
5. state 소스: FrameAlignedState vs joint_states

---

## 12. 한 줄 요약

- **3-레이어**: (1) 입력/구동 → (2) 정책 추론·실행(완료) → (3) **Episode Manager 상태머신(신규)**.
- Manager가 **홈 복귀 ↔ 정책 실행을 게이트로 상호배제**하며 `IDLE→HOMING→READY→RUNNING→STOPPING→HOMING`을 돌린다.
- 친구는 **카메라2 + state(26) + 안전센서**를 계약된 토픽으로 발행, 우리는 **set_enable 게이트 + Episode Manager**를 만들면 완성.
