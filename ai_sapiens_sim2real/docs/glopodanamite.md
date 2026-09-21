# Glopodanamite: localization 기반 sim2sim / sim2real

`Cyclo-Mimic-K1-Rev1-Dynamite-Gloposition`의 ONNX를 기존 K1 mimic runtime에서
실행한다. sim2sim과 sim2real 모두 `ai_sapiens_private`의 `feature-localization`
브랜치가 발행하는 `/state_estimator/odom`을 로봇 XY의 입력으로 사용한다.

춤을 추면서 이동·회전을 조작하는 selector 204 버전은
[glopodanamite_controller](glopodanamite_controller.md)를 참고한다.
최신 controller MoE는 selector **205**이며
[MoE sim2sim 문서](glopodanamite_controller_moe.md)에 연결 모델과 실행 절차를 정리했다.

## 정책과 모션

기존 asset 구조를 그대로 사용한다.

```text
assets/k1/mimic/glopodanamite/
├── exported/policy.onnx
└── params/
    ├── sim2real.yaml
    ├── dynamite004_headwrap_v3.csv
    ├── env.yaml
    └── agent.yaml
```

실행에 필요한 파일은 ONNX, `sim2real.yaml`, CSV 세 개다. `env.yaml`과
`agent.yaml`은 학습 설정을 확인하는 자료다. assets는 현재 Git ignore 대상이므로
코드 배포 시 이 폴더도 별도로 복사해야 한다. 로컬 `sim2real.yaml`의
`actions.joint_pos.raw_clip`에 23개 관절 각각 `[-10, 10]`을 추가했다.
재수출한 YAML로 교체할 때에도 학습의 `clip_actions=10`을 유지해야 한다.

`config/k1_config.yaml`에 selector **203**, 상태 **MimicGlopodanamite**,
behavior **mimic_glopodanamite**를 등록했다. 모션은 50 FPS, 9,757 프레임이며
완료되면 기존 기본값인 `Velocity`로 이동한다. `motion_format`, `fps`는 이
behavior에 생략할 수 있다. FPS 기본값은 50이고, 형식을 생략한 global-position
정책은 MJLab 방식으로 프레임을 보간하고 관절 속도를 중앙 차분으로 계산한다.
기존 global-position 관측이 없는 모션은 legacy 처리를 유지한다. 명시적인
`motion_format: legacy|mjlab` 설정은 자동 선택보다 우선한다.

관측 순서는 제공된 YAML과 ONNX의 128차원 입력에 맞춘다. 출력은 23차원이다.

| Python slice | 관측 | 데이터 |
| --- | --- | --- |
| `[0:46]` | `motion_command` | CSV 관절 위치와 속도 |
| `[46:52]` | `motion_anchor_ori_b` | 추정기 root 자세와 측정 waist 관절로 계산한 torso 기준 reference 방향 |
| `[52:55]` | `base_ang_vel` | 기존 IMU 각속도 |
| `[55:78]` | `joint_pos_rel` | 측정 관절 위치 − 학습 기본 자세 |
| `[78:101]` | `joint_vel_rel` | 측정 관절 속도 |
| `[101:124]` | `last_action` | 직전 raw action에 학습과 동일한 clip 적용 |
| `[124:126]` | `robot_root_position_xy_w` | 미믹 진입 위치 대비 이동량, 모션 방향으로 정렬 |
| `[126:128]` | `reference_root_position_xy_w` | CSV root XY − 미믹 시작 프레임의 root XY |

## 좌표계

추정기는 로봇 세션 시작 시 실행·초기화하고 보행과 춤 사이에도 계속 유지한다.
미믹 진입 순간의 odom 위치와 yaw, 모션 시작 프레임의 위치와 yaw를 저장한다.
기본값 `localization_align_on_entry:=true`에서 관측을 다음처럼 계산한다.

```text
odom_start_xy = odom_xy_at_entry
reference_start_xy = reference_xy_at_time_start
R = yaw_rotation(reference_root_yaw_at_entry - odom_root_yaw_at_entry)
robot_xy_for_policy = R * (current_odom_xy - odom_start_xy)
reference_xy_for_policy = current_reference_xy - reference_start_xy
robot_root_orientation_for_policy = R * current_odom_root_orientation
```

미믹 첫 프레임의 로봇 XY와 reference XY는 모두 `(0, 0)`이다. 이후 같은
고정 offset을 사용하므로 이동량과 경로 오차가 남는다. 보행·회전 후 다시
미믹을 선택하면 새 위치와 yaw를 캡처한다. estimator의 odom, yaw, timestamp는
초기화하거나 수정하지 않는다. `time_start`가 0이 아니면 CSV 첫 줄이 아닌
그 재생 시작 시점의 위치를 reference 원점으로 잡는다.

XY의 축은 모션 파일의 월드 축을 유지한다. 로봇의 시작 방향만 reference의
시작 방향에 맞추며 `motion_anchor_ori_b`에도 같은 yaw 변환을 적용한다.
보행 중 변한 위치·방향을 새 춤의 기준으로 맞추는 처리이며 추정기의 드리프트를
교정하는 기능은 아니다.

`localization_align_on_entry:=false`는 이 변환을 끈다. 이 경우 odom XY와 자세,
CSV reference XY를 직접 사용하므로 외부에서 두 좌표계가 일치해야 한다.

### 학습 좌표와의 차이

확인한 cyclo_mjlab `main`의 XY 관측은 환경 배치 원점만 제거한 로봇 월드 XY와
CSV 원본 reference XY다. 모션 시작점을 빼는 학습 관측은 아니다. 제공 CSV의
첫 XY는 약 `(0.053737, 0.019352)`이므로 이번 변경은 기존 정렬 방식에 비해
로봇과 reference 입력 모두에서 이 값을 뺀다. 두 위치의 차이는 유지되지만
절대 XY를 받는 신경망의 출력이 같다는 보장은 없다. 기존 ONNX를 수정하거나
재학습하지 않았으며, 합성 센서 테스트는 좌표 처리·재진입·추론 실행을 검증한다.
실제 모션 안정성을 확인하고, 동일한 관측 정의로 학습과 배포를 맞추려면
학습 측에서도 같은 원점 정규화를 적용한 정책을 준비해야 한다.

수신 메시지는 `nav_msgs/msg/Odometry`, `header.frame_id=odom`,
`child_frame_id=pelvis`여야 한다. 유한한 pose, 유효한 quaternion, 증가하는
timestamp를 검사한다. 유효한 동일 timestamp의 반복 메시지는 건너뛰며,
마지막 정상 위치나 수신 시간을 갱신하지 않는다. 수신 시각과 메시지 시각 모두 기본 0.5초 이내여야 한다.
센서와 정책 노드는 같은 ROS 시간 기준을 사용해야 한다.

localization 없이도 Damping/ReadyPose와 기존 정책은 사용할 수 있다.
이 정책은 localization이 없거나 유효하지 않으면 미믹 추론을 중단하고
`authority.default_velocity_state`에 설정된 `Velocity`로 전환한다. 실행 중
새 timestamp가 0.5초 동안 들어오지 않거나 잘못된 메시지가 감지되어도 같은
동작을 적용한다. 전환 tick에서 Velocity 정책을 초기화하고 추론한 후 명령을
발행한다. 단순 중복 메시지로는 미믹을 종료하지 않는다. 데이터 복구 후 미믹을
다시 선택하여 진입한다. 조종 입력 상실 등 기존의 별도 Damping 조건은 유지한다.

## 빌드

ROS 2 Jazzy 및 기존 패키지 의존성, ONNX Runtime이 필요하다. private 추정기는
추가로 Pinocchio와 GTSAM을 요구한다. private 저장소가 `feature-localization`에
있는지 확인하고 아래처럼 **추정기 패키지만** 추가하여 빌드한다.

```bash
cd /home/kim/ai_sapiens
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install \
  --base-paths /home/kim/ai_sapiens/ai_sapiens \
               /home/kim/ai_sapiens_private/ai_sapiens_state_estimator \
  --packages-up-to ai_sapiens_bringup ai_sapiens_mujoco \
                   ai_sapiens_sim2real ai_sapiens_state_estimator
source install/setup.bash
```

ONNX Runtime이 표준 경로에 없다면 `--cmake-args -DONNXRUNTIME_ROOT=<설치 경로>`를
추가한다. private 저장소 전체를 `--base-paths`에 추가하면 중복 이름의
interfaces/description 패키지가 발견될 수 있다. 추정기가 요구하는
`FootContactState.msg`를 현재 public `ai_sapiens_interfaces`에 동일하게 추가했으므로
이 인터페이스를 함께 재빌드한다.

## 실행

각 터미널에서 위 workspace의 `install/setup.bash`를 source한다.
sim2sim에서는 기존 MuJoCo backend를 실행한다.

```bash
ros2 launch ai_sapiens_bringup k1_mujoco.launch.py
```

sim2real에서는 위 명령 대신 기존 하드웨어 backend를 실행한다.

```bash
ros2 launch ai_sapiens_bringup k1.launch.py
```

로봇 세션 시작 시 별도 터미널에서 추정기와 정책 노드를 함께 실행한다. 키보드 조작 예시는 다음과 같다.

```bash
ros2 launch ai_sapiens_sim2real glopodanamite.launch.py \
  teleop_input_plugin:=ai_sapiens_sim2real/KeyboardTeleopInputPlugin \
  teleop_input_config_path:="$(ros2 pkg prefix ai_sapiens_sim2real)/share/ai_sapiens_sim2real/config/teleop/keyboard.yaml"
```

별도 대화형 터미널에서 키보드 입력 노드를 실행한다.

```bash
ros2 run ai_sapiens_sim2real keyboard_teleop_node
```

`2`로 ReadyPose에 진입하고 로봇을 정지시킨 상태에서 추정기의 기존 활성화
서비스를 세션 시작 때 한 번 호출한다. 초기 IMU bias 수집이 완료될 때까지 움직이지 않는다.
MuJoCo 기본 설정은 gantry가 켜져 있으므로 모션 재생 전 기존 viewer 조작으로
지면 접촉 상태를 준비한다.

```bash
ros2 service call /state_estimator_node/enable_inekf_orientation std_srvs/srv/Empty "{}"
ros2 topic echo /state_estimator/odom --once
```

`3`으로 보행한 뒤, 키보드 좌우 방향키로 `MimicGlopodanamite (203)`을 선택하고
`4`로 춤을 시작할 수 있다. 춤 사이에 estimator를 재실행하거나 서비스를 다시
호출하지 않는다. 미믹 진입마다 위치·방향 offset이 자동으로 새로 잡힌다.
`1`은 Damping, `2`는 ReadyPose다. RadioMaster를 사용하면 teleop override를
생략한다. 기존 CH11 값 `1060`이 selector 203이며 CH7/CH6의 기존 mimic 조작을
사용한다. MuJoCo에서 RadioMaster USB를 쓸 경우 backend에
`radiomaster_usb:=true`를 추가한다.

추정기를 이미 별도로 실행 중이라면 wrapper 대신
`ai_sapiens_sim2real.launch.py robot:=k1`을 사용한다. 이 launch에서
`localization_topic`, `localization_world_frame`, `localization_base_frame`,
`localization_timeout`, `localization_align_on_entry`를 조정할 수 있다.

## 검증

신규 단위 테스트는 좌표 정렬, XY/방향 관측, MJLab 모션 보간 및 속도,
action clip, odometry frame/timestamp/timeout 처리를 검증한다.
`scripts/run_gloposition_smoke_test.py`는 실제 제공 ONNX를 로드하여 입력 128개,
localization 이동 반영, 중복 timestamp 허용, 데이터 누락 및 timestamp 정지 시
Velocity 전환, 보행·회전 후 재진입 시 두 XY의 0 시작 및 새 방향 적용을 검증한다.

```bash
python3 /home/kim/ai_sapiens/ai_sapiens/ai_sapiens_sim2real/scripts/run_gloposition_smoke_test.py
```

이 테스트는 localhost의 별도 ROS domain 187에서 합성 센서를 발행하고, 명령을
테스트 토픽으로만 보낸다. 실제 MuJoCo 물리 동작, private 추정기의 추정 정확도,
하드웨어 모션 성공 여부는 이 테스트의 검증 범위에 포함되지 않는다.
