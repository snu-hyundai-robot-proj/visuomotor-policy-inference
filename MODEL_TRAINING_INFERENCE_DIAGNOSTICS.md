# Model Training/Inference Diagnostics

## 결론

현재 pretrained config에는 `observation.gripper_sensor`와 `observation.wrist_ft_sensor`가 정의되어 있다. 그러나 현재 `DiffusionPolicy` implementation 기준으로는 이 두 sensor feature가 training loss나 inference action generation의 conditioning에 실제로 사용되지 않는 것으로 판단된다.

즉, “training에서는 tactile/sensor를 사용하고 inference에서는 누락”된 구조라기보다는, config와 normalizer에는 sensor feature가 남아 있지만 model forward path에서는 사실상 사용되지 않는 구조에 가깝다.

## 확인된 Feature

현재 policy config:

- `observation.images.front_rgb`: `3 x 480 x 640`
- `observation.images.wrist_rgb`: `3 x 480 x 640`
- `observation.state`: `26`
- `observation.gripper_sensor`: `30`
- `observation.wrist_ft_sensor`: `6`
- `action`: `26`

`observation.state`는 arm 6 DoF + hand 20 DoF의 26D joint vector이며, gripper/FT sensor 값을 포함하지 않는다.

## Sensor Feature가 실제 Model에 안 들어가는 이유

`DiffusionModel._prepare_global_conditioning()`은 다음 feature만 global conditioning으로 concatenate한다.

- `observation.state`
- image features
- `observation.environment_state`
- `observation.tactile*`

하지만 현재 sensor key는 다음과 같다.

- `observation.gripper_sensor`
- `observation.wrist_ft_sensor`

이 key들은 `observation.tactile*` prefix가 아니고, `environment_state`도 아니다. 또한 `robot_state_feature`는 정확히 `observation.state`만 선택한다. 따라서 sensor feature가 config에는 있어도 model conditioning에는 포함되지 않는다.

## Training과 Inference의 의미

Training의 `compute_loss()`와 inference의 `generate_actions()`는 모두 같은 `_prepare_global_conditioning()` 경로를 사용한다. 따라서 sensor feature는 training과 inference 양쪽에서 모두 사용되지 않았을 가능성이 높다.

Preprocessor/normalizer는 sensor feature를 알고 있지만, 입력 dict에 해당 key가 없으면 normalize하지 않고 넘어간다. 즉, FastAPI inference에서 sensor 값이 빠져도 바로 에러가 나지 않는다. 그러나 이것은 sensor가 정상적으로 사용된다는 뜻이 아니라, model이 애초에 해당 key를 conditioning으로 소비하지 않는다는 뜻이다.

## 코드상 문제점

1. Config와 model forward path의 불일치

   Config에는 `gripper_sensor`, `wrist_ft_sensor`가 있지만 DiffusionPolicy conditioning에는 반영되지 않는다. 문서와 실험 로그에서 sensor를 사용했다고 해석하면 잘못된 결론이 될 수 있다.

2. ROS state 구성 위험

   ROS LeRobot node는 `state_fields`를 concat한 뒤 required state dim인 26D에 맞춰 truncate한다. 기본 `state_fields`는 `robot_joint`, `target_robot_joint`, `robot_pose`, `robot_ft`, `gripper_joint`, ... 순서이다. 기본값 그대로라면 `observation.state`가 arm 6 + hand 20이 아니라 앞 26개 값으로 잘릴 수 있다. 이는 policy가 기대하는 state semantics와 다를 수 있다.

3. Camera key mismatch 가능성

   Policy config는 `observation.images.front_rgb`, `observation.images.wrist_rgb`를 기대한다. 일부 ROS/UI 코드에서는 `observation.images.d405`, `observation.images.zivid` 같은 key가 보인다. 실제 실행 시 camera key가 config와 다르면 image observation이 제대로 들어가지 않는다.

4. Sensor를 사용하려면 재설계 필요

   Sensor를 실제로 쓰려면 key naming과 model conditioning을 정리해야 한다. 예를 들어 sensor feature를 `observation.tactile...` 규칙에 맞추거나, `DiffusionPolicy`가 `gripper_sensor`, `wrist_ft_sensor`를 global conditioning에 명시적으로 concatenate하도록 수정한 뒤 재학습해야 한다.

## 권장 조치

- 제출/보고 문서에는 core model을 `front/wrist RGB + 26D joint state -> 26D joint action`으로 설명한다.
- Sensor feature는 “config에 존재하지만 현재 DiffusionPolicy conditioning에서는 사용되지 않는 항목”으로 분리해서 기록한다.
- ROS inference를 사용할 경우 `state_fields`를 `robot_joint + gripper_joint`만으로 맞추거나, policy가 학습한 `observation.state` 구성과 정확히 일치하도록 수정한다.
- Camera keys를 반드시 `observation.images.front_rgb`, `observation.images.wrist_rgb`로 맞춘다.
