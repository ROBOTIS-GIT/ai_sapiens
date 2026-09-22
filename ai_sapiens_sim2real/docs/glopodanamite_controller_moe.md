# Glopodanamite controller MoE sim2sim

`Cyclo-Mimic-K1-Rev1-Dynamite-Gloposition-controller-moe`를 기존 MuJoCo backend와
K1 mimic runtime에서 실행한다. selector는 **205**, 상태는
`MimicGlopodanamiteControllerMoe`다. 기존 203/204 정책도 유지한다.

## 기준 학습 코드

cyclo_mjlab의 위치 추종 구현(`ff224ff`)을 기준으로 한다.
`dynamite_moe_env_cfg.py` → `dynamite_controller_env_cfg.py` →
`dynamite_gloposition_env_cfg.py`의 actor 관측과 `mdp/steering.py`의 누적 참조
변환을 C++ runtime에 대응한다. 명령 해제 시에는 아래의 심투심 전용 보정을 적용한다.
실제 학습은 별도 환경에서 수행할 수 있으며,
이 문서는 특정 체크포인트의 학습 상태를 전제하지 않는다.

ONNX, `sim2real.yaml`, `env.yaml`, `agent.yaml`은 같은 학습 실행에서 export한 것을
사용한다. 위치 정책과 속도 정책은 모두 입력이 131개일 수 있다. runtime은
ONNX의 `joint_names`·`observation_names`를 YAML 순서와 비교하고, 과거의
`robot/reference_root_velocity_xy_h` 또는 `tracking_mode: velocity` 설정을 거부한다.

## 배포 구조

MoE의 base, translation expert, rotation expert, soft gate와 residual scale이
**하나의 ONNX에 포함**되어 있다. C++에서 expert 선택이나 residual을 추가 적용하지
않고 ONNX를 한 번 실행한다. actor 입력 131개, 출력 23개이며 기존 controller의
관측 순서와 action 처리 방식으로 실행할 수 있다.

| 항목 | 처리 |
| --- | --- |
| `[0:46]` | 원본 CSV 관절 위치·중앙 차분 속도 |
| `[46:52]` | estimator 자세와 waist 관절로 계산한 상대 anchor 방향, steering yaw 반영 |
| `[52:124]` | IMU 각속도, 관절 위치·속도, 직전 clip된 action |
| `[124:126]` | 춤 진입 시 estimator XY/yaw를 기준으로 정렬한 로봇 XY |
| `[126:128]` | CSV XY − 시작 XY + 누적 steering 이동량 |
| `[128:131]` | 필터를 거친 추가 vx, vy, yaw rate |
| 주기 | 정책 50 Hz, CSV 50 FPS, 물리 0.005초 × 4회 |
| 명령 | vx/vy ±0.3 m/s, yaw rate ±0.3 rad/s, 필터 시간상수 0.5초 |
| action | raw ±10 clip → YAML의 관절별 scale/offset 적용 |

학습 코드의 footstep/IK 목표는 보상 계산에 사용된다. actor의 `motion_command`는
여전히 원본 모션 관절 위치·속도이므로 배포 관측을 IK 목표로 바꾸지 않는다.
모션 양 끝의 관절 속도는 단방향 차분, 내부 프레임은 중앙 차분을 사용한다.

학습 설정의 `observation_origin: episode`에 맞춰 시작점을 제거한다.
estimator는 세션 내내 유지하며,
보행·회전 후 미믹 진입마다 로봇 XY/yaw와 reference 시작 XY를 새로 캡처한다.
첫 관측의 두 XY와 적용 속도는 0이다. 학습의 reset 노이즈는 배포에 추가하지 않는다.
`localization_align_on_entry:=true`를 유지한다.

추가 이동 명령은 측정한 로봇 pelvis heading으로 회전시켜 누적한다. 원본 모션의
프레임 간 이동량은 steering yaw의 중간값으로 회전한다. 배포 runtime은
`commands.reference_trajectory.steering`이 있는 모든 미믹에 다음 조종 규칙을
기본 적용한다. 별도 설정 옵션 없이 controller 204와 MoE 205에 적용하며,
steering이 없는 일반 미믹에는 적용하지 않는다. 각 축의 실제 요청 명령 절댓값이
0.1 이하이면 0으로 처리한다(XY는 m/s, yaw는 rad/s). ±0.1도 포함하며,
0.1을 초과하는 명령은 크기를 재조정하지 않고 그대로 사용한다. 정규화된
조종기 축이 아닌 정책 범위로 변환된 속도에 적용한다. 이 중립 판정을 적분과
해제 처리에 동일하게 사용하며, 이전 명령의 필터 감속은 유지한다.
이동(XY)과 회전 해제는 독립적이다. 회전 중에도 이동을 해제할 수 있고,
이동만 조종했다면 해제할 때 춤의 heading을 덮어쓰지 않는다.

해제된 명령은 기존 필터로 감속하면서 참조를 도달한 pose에 맞춘다.
목표가 1m, 실제 도달 위치가 0.7m라면 남은 0.3m를 버린다. 해제할 때
참조가 바뀌는 것은 의도한 동작이다. 천천히 오차를 지우면 그동안 기존
목표를 따라잡으려 하므로 그렇게 처리하지 않는다. 필터의 감속과 실제
로봇의 관성 때문에 해제 즉시 물리적으로 정지한다는 보장은 없다.
관절 모션과 재생 시간, 루트 높이·기울기, odom 좌표계는 계속 유지한다.

적용 평면 속도 또는 회전 속도가 0.01m/s 또는 0.01rad/s 이하이면 해당
필터의 잔여 속도를 0으로 정리한다. 해당 해제 처리가 끝나면 도달한
위치·방향을 기준으로 원본 춤 동선을 계속 진행하며, 매 주기 로봇을 따라
참조를 옮기는 처리를 끝낸다. 새 명령은 해당 이동/회전 해제 보정을 취소한다.
처음부터 명령이 0인 경우에는 원본 동선을 그대로 추종하며, 재진입 시에는
누적값과 해제 상태를 모두 초기화한다.

조종 중에는 기존 학습 방식대로 목표를 적분하므로 추종 실패가 누적될 수 있다.
원본 춤의 추종 오차나 estimator 드리프트도 해제 처리로 항상 제거되는 것은 아니다.
위치 추종 오차만으로 Damping으로 자동 전환하거나 추론을 중단하지 않는다.
키보드 UI의 속도는 누적 설정값이며 키에서 손을 떼도 유지된다. `Space`로
명령을 0으로 만들어야 이 해제 처리가 작동한다.

이 처리는 배포 C++ runtime의 기본 동작이며 학습 코드와 ONNX/YAML은 수정하지 않는다.
따라서 작은 명령을 무시하는 규칙과 해제 순간의 목표 갱신은 학습 코드와 다르다.

공통 K1 MuJoCo XML도 학습의 기본 물리 설정을 사용한다: implicitfast,
0.005초 timestep, solver iterations 10, line-search iterations 20, CCD iterations 50.
`FULL_COLLISION`과 같이 발 접촉은 `condim=3`, `priority=1`, 마찰 0.6이고
나머지 로봇 충돌은 `condim=1`이다. 이 XML 변경은 K1의 다른 정책에도 적용된다.
학습용 마찰 무작위화·센서 노이즈·외란은 배포에서 재현하지 않는다.

로봇 XY는 기존 요청대로 `/state_estimator/odom`에서 받는다. localization이
없거나 유효한 새 데이터가 timeout 동안 없으면 미믹을 종료하고 Velocity로 전환한다.

## 파일 배치

```text
assets/k1/mimic/glopodanamite_controller_moe/
├── exported/policy.onnx
├── source_manifest.yaml
└── params/
    ├── sim2real.yaml
    ├── env.yaml
    ├── agent.yaml
    └── dynamite004_headwrap_v3.csv
```

K1 설정은 기존 미믹과 같은 형식이다. steering과 observation 설정은 export된
`params/sim2real.yaml`을 그대로 사용한다.

```yaml
mimic_glopodanamite_controller_moe:
  kind: mimic
  asset: mimic/glopodanamite_controller_moe
  motion: dynamite004_headwrap_v3.csv
```

**assets는 Git ignore 대상**이다. 다른 환경에는 이 폴더를 함께 복사해야 한다.
새 export를 가져올 때는 NumPy, PyYAML, Python ONNX Runtime이 있는 환경에서
다음 도구를 사용할 수 있다. 실행 폴더·관측 순서·좌표 원점·steering·joint order를
검사하고 파일별 SHA-256과 출처를 기록한다. 기존 출력 폴더를 덮어쓰지 않으므로
다른 모델을 검토할 때는 `--output <새 폴더>`를 지정한다.

```bash
python3 ai_sapiens_sim2real/scripts/import_mimic_moe_assets.py \
  --run-dir /path/to/copied-training-run \
  --motion-csv /path/to/dynamite004_headwrap_v3.csv \
  --output /path/to/new-policy-bundle
```

## 빌드와 실행

private `feature-localization` 추정기가 준비된 기존 컨테이너에서 정책과 asset을
다시 설치한다. 처음 구축하는 환경은 [의존성과 추정기 빌드](glopodanamite.md#빌드)를 따른다.

```bash
cd /root/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-up-to ai_sapiens_sim2real ai_sapiens_description ai_sapiens_mujoco
source install/setup.bash
```

각 터미널에서 workspace를 source한 뒤 실행한다.

```bash
# 터미널 1: MuJoCo backend
ros2 launch ai_sapiens_bringup k1_mujoco.launch.py
```

```bash
# 터미널 2: estimator와 정책, 키보드 입력 사용
ros2 launch ai_sapiens_sim2real glopodanamite.launch.py \
  teleop_input_plugin:=ai_sapiens_sim2real/KeyboardTeleopInputPlugin \
  teleop_input_config_path:="$(ros2 pkg prefix ai_sapiens_sim2real)/share/ai_sapiens_sim2real/config/teleop/keyboard.yaml" \
  debug_publish_enabled:=true
```

```bash
# 터미널 3: 키보드 조작
ros2 run ai_sapiens_sim2real keyboard_teleop_node
```

`2`로 ReadyPose를 선택하고 로봇을 정지시킨 뒤, 세션 시작 시 한 번 estimator를
활성화한다. IMU bias 수집이 끝나고 odom이 나오는 것을 확인한다.

```bash
ros2 service call /state_estimator_node/enable_inekf_orientation std_srvs/srv/Empty "{}"
ros2 topic echo /state_estimator/odom --once
```

기본 gantry가 켜져 있으므로 viewer에서 기존 방식으로 지면 접촉 상태를 준비한다.
키보드 좌우 방향키로 **205 / MimicGlopodanamiteControllerMoe**를 선택하고 `4`로
진입한다. `w/s` 전후, `a/d` 좌우, `q/e` 회전이며 스페이스로 추가 명령을 0으로
만든다. `3`은 Velocity로 전환한다. estimator는 춤마다 재시작하지 않는다.

RadioMaster를 쓰면 policy launch의 키보드 override를 생략하고, MuJoCo launch에
`radiomaster_usb:=true`를 추가한다. **CH11 1100 → selector 205**이며 기존 mimic
진입 조작을 사용한다. DualSense selector 목록에도 205가 자동으로 표시된다.
추정기가 이미 실행 중이면 wrapper 대신 `ai_sapiens_sim2real.launch.py robot:=k1`을
사용하여 중복 실행을 피한다.

## 검증

workspace를 source한 후 다음 명령으로 실제 MoE ONNX와 C++ runtime을 검사한다.

```bash
python3 ai_sapiens_sim2real/scripts/run_gloposition_smoke_test.py --moe
python3 ai_sapiens_sim2real/scripts/run_gloposition_smoke_test.py --moe --teleop dualsense
python3 ai_sapiens_sim2real/scripts/run_gloposition_smoke_test.py --controller
```

합성 센서와 별도 localhost ROS domain에서 131개 obs, 조종 입력·필터·reference
보정, 보행·회전 후 재진입 원점, localization timeout 시 Velocity 전환을 확인한다.
실제 MuJoCo 물리 안정성이나 private estimator의 정확도를 검증하는 테스트는 아니다.

실제 실행에서 조종 입력과 위치 오차는 읽기 전용 도구로 확인할 수 있다.
DualSense, 키보드, RadioMaster 원시 입력과 적용 명령, 현재/최근 최대 XY 오차를
표시한다. `debug_publish_enabled:=true`로 실행한 정책에서 사용한다.

```bash
python3 ai_sapiens_sim2real/scripts/inspect_mimic_steering.py
```

`mode=MimicGlopodanamiteControllerMoe`, `obs_size=131`, 마지막 세 관측인 `applied`와
reference XY를 확인한다. 관측·명령 경로 검증과 실제 학습 모델의 물리 추종 성능
평가는 별도로 수행한다.
