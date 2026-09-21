# Glopodanamite controller MoE sim2sim

`Cyclo-Mimic-K1-Rev1-Dynamite-Gloposition-controller-moe`를 기존 MuJoCo backend와
K1 mimic runtime에서 실행한다. selector는 **205**, 상태는
`MimicGlopodanamiteControllerMoe`다. 기존 203/204 정책도 유지한다.

## 가져온 정책과 학습 상태

이번에 연결한 파일은 cyclo_mjlab의
`logs/rsl_rl/k1_mimic_moe/2026-09-17_08-54-40` 실행에서 가져왔다.
`exported/policy.pt`의 30개 파라미터·버퍼가 `model_29999.pt`의 actor와 모두 일치했다.
같은 실행의 `policy.onnx`, `sim2real.yaml`, `env.yaml`, `agent.yaml`과
학습에 사용한 `dynamite004_headwrap_v3.csv`를 별도 MoE asset에 복사했다.

**이 체크포인트는 curriculum level 0(속도 명령이 모두 0인 춤 학습 단계)이다.**
translation/rotation expert의 마지막 layer weight와 bias는 모두 0이다.
다른 관측을 고정하고 속도 3개만 바꾸면 출력 action은 동일하다.
reference XY/방향이 바뀌면 base 정책은 반응할 수 있지만, 이동·회전 expert가
학습된 모델로 볼 수는 없다. 조종 동작을 평가할 때 입력 전달 여부와 이 학습
상태를 구분해야 한다. 학습 체크포인트나 curriculum은 여기서 수정하지 않는다.

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
| 주기 | 50 Hz, CSV 50 FPS, 9,757 프레임 |
| 명령 | vx/vy ±0.3 m/s, yaw rate ±0.3 rad/s, 필터 시간상수 0.5초 |
| action | raw ±10 clip → YAML의 관절별 scale/offset 적용 |

학습 코드의 footstep/IK 목표는 보상 계산에 사용된다. actor의 `motion_command`는
여전히 원본 모션 관절 위치·속도이므로 배포 관측을 IK 목표로 바꾸지 않는다.
CSV 관절 순서를 NPZ 순서로 대응시켜 비교했으며 최대 차이는 위치 약 `5e-10`,
중앙 차분 속도 약 `1.2e-6`이었다.

이 실행은 학습 YAML에도 `observation_origin: episode`가 명시되어 있다.
현재 runtime의 시작점 제거와 맞는 정책이다. estimator는 세션 내내 유지하며,
보행·회전 후 미믹 진입마다 로봇 XY/yaw와 reference 시작 XY를 새로 캡처한다.
첫 관측의 두 XY와 적용 속도는 0이다. 학습의 reset 노이즈는 배포에 추가하지 않는다.
`localization_align_on_entry:=true`를 유지한다.

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
  --run-dir /home/kim/cyclo_mjlab/cyclo_mjlab/logs/rsl_rl/k1_mimic_moe/2026-09-17_08-54-40 \
  --motion-csv /home/kim/cyclo_mjlab/cyclo_mjlab/source/assets/motions/K1_rev1/dynamite_gloposition/dynamite004_headwrap_v3.csv
```

## 빌드와 실행

private `feature-localization` 추정기가 준비된 기존 컨테이너에서 정책과 asset을
다시 설치한다. 처음 구축하는 환경은 [의존성과 추정기 빌드](glopodanamite.md#빌드)를 따른다.

```bash
cd /root/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-up-to ai_sapiens_sim2real
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
```

합성 센서와 별도 localhost ROS domain에서 131개 obs, 조종 입력·필터·reference
보정, 보행·회전 후 재진입 원점, localization timeout 시 Velocity 전환을 확인한다.
실제 MuJoCo 물리 안정성이나 private estimator의 정확도를 검증하는 테스트는 아니다.

실제 실행에서 조종 입력이 전달되는지는 읽기 전용 도구로 확인할 수 있다.

```bash
python3 ai_sapiens_sim2real/scripts/inspect_mimic_steering.py
```

`mode=MimicGlopodanamiteControllerMoe`, `obs_size=131`, 마지막 세 관측인 `applied`와
reference XY를 확인한다. 모델의 학습 상태 때문에 입력 경로가 정상이어도 원하는
춤·이동·회전 추종이 보장되지는 않는다.
