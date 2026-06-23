# system_ui_lerobot 사용 가이드

이 패키지는 좌/우 시스템을 한 UI에서 묶어서 관리하는 오케스트레이터입니다. 텔레옵과 데이터 저장 흐름은 제외했고, 현재는 아래 3가지만 다룹니다.

- 로봇 시스템 연결 / 정지
- Vision 시스템 연결 / 정지
- LeRobot inference 실행 / 결과 출력 전용 전환 / init pose

## 실행 순서

1. 워크스페이스를 빌드하고 `install/setup.bash` 를 source 합니다.
2. `system_ui` 를 실행합니다.
3. 좌측 또는 우측 패널에서 필요한 시스템을 순서대로 켭니다.
4. LeRobot 패널에서 policy path 를 확인한 뒤 inference 를 시작합니다.
5. 필요하면 mode 버튼으로 `실제 action 수행` / `추론 결과 출력만` 을 전환합니다.
6. `Init Pose` 로 초기 자세를 요청하고, `Stop` 으로 해당 side 를 정지합니다.

## 패널 역할

### Robot

- `Connect`: 해당 side 의 로봇 시스템을 시작합니다.
- `Stop`: 해당 side 의 로봇 시스템을 종료합니다.

### Vision

- `Connect`: 해당 side 의 Vision 노드를 시작합니다.
- `Stop`: 해당 side 의 Vision 노드를 종료합니다.

### LeRobot

- `Inference`: LeRobot 추론 노드를 시작합니다.
- `Mode: Execute Action` / `Mode: Print Only`: 추론 결과를 실제 action 으로 보낼지, 로그/출력만 볼지 전환합니다.
- `Init Pose`: 해당 side 의 초기 자세 요청을 보냅니다.
- `Stop`: LeRobot 노드를 종료합니다.

## 동작 방식

- 로봇 쪽 init pose 는 `system_left` / `system_right` 로 `UiCommand(command=0, value=1)` 을 보냅니다.
- LeRobot 는 `lerobot_system_left` / `lerobot_system_right` 를 띄워서 `FrameAlignedState` 와 카메라 입력을 받아 추론합니다.
- 카메라 입력은 side 별 d405 / zivid 조합을 전제로 합니다.

## 주의 사항

- LeRobot 패널의 policy path 는 실제 모델 경로로 바꿔야 합니다.
- 실제 카메라 토픽 이름은 현재 시스템의 퍼블리셔 이름과 맞아야 합니다.
- mode 를 바꿀 때는 실행 중인 LeRobot 프로세스가 재시작되어 설정이 즉시 반영됩니다.

## 한 줄 요약

- 로봇과 Vision 을 켠 뒤 LeRobot 를 실행하고, mode 를 `Execute Action` 으로 두면 실제 동작, `Print Only` 로 두면 추론 결과 확인용입니다.
