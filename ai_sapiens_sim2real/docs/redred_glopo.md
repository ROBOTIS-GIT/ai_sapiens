# Redred 글로벌 XY mimic: sim2sim / sim2real

`Cyclo-Mimic-K1-Rev1-Redred` 정책을 **selector 206 / MimicRedredGlopo**로 실행한다.
제공된 ONNX는 cyclo_mjlab의 `k1_mimic_redred/2026-09-21_02-03-47` export와
SHA-256이 일치한다. 기존 glopodanamite 계열과 같은 K1 runtime을 사용한다.

## 정책과 설정

```yaml
mimic_redred_glopo:
  kind: mimic
  asset: mimic/redred_glopo
  motion: redred_k1_conver.csv
```

`k1_config.yaml`은 기존처럼 `kind`, `asset`, `motion`만 지정한다.
기본 50 FPS와 MJLab 모션 처리를 사용하며, 1,710프레임의 재생 구간은
0~34.18초다. 완료되면 기본 상태인 `Velocity`로 전환한다.

```text
assets/k1/mimic/redred_glopo/
├── exported/policy.onnx
└── params/
    ├── sim2real.yaml
    └── redred_k1_conver.csv
```

관측은 128개, action은 23개다. 관절·anchor 방향·IMU·이전 action 124개 뒤에
로봇 XY 2개와 reference XY 2개를 넣는다. **이 모델에는 조종기의 추가 이동·회전
속도 입력이 없다.** controller/MoE용 steering 설정을 추가하지 않는다.
조종기로 상태 선택과 춤 시작·종료를 할 수 있다.

CSV와 학습 원본 CSV가 동일하며, NPZ의 관절 순서를 맞춰 비교한 위치 차이는
최대 약 `5e-10`, 중앙 차분 관절 속도 차이는 약 `3.2e-6`이었다.
정책 주기는 `step_dt: 0.02`다. 제공된 YAML에 빠져 있던
`actions.joint_pos.raw_clip`을 학습 run의 `agent.yaml`에 있는 `clip_actions: 10.0`과
같이 23개 `[-10, 10]`으로 추가했다. clip 후 관절별 scale/offset을 적용한다.
모델 가중치는 변경하지 않았다.

## 글로벌 위치와 시작점

sim2sim과 sim2real 모두 private `feature-localization` 추정기의
`/state_estimator/odom`을 사용한다. 기본 `localization_align_on_entry:=true`에서
미믹 진입 시 위치와 yaw를 저장하고 다음과 같이 관측을 만든다.

```text
R = yaw_rotation(reference_start_yaw - estimator_start_yaw)
robot_xy = R * (estimator_xy - estimator_start_xy)
reference_xy = csv_xy - csv_start_xy
```

시작 관측은 두 XY 모두 `(0, 0)`이며, 보행·회전 후 다시 춤을 시작하면 기준을
새로 잡는다. estimator는 계속 실행한다. 유효한 localization이 없거나 정상 데이터가
timeout 동안 없으면 미믹을 종료하고 `Velocity`로 전환한다.

**학습과의 차이:** 확인한 Redred 학습은 환경 배치 원점만 제거하고 CSV 시작점은
빼지 않는다. CSV 첫 XY는 약 `(-0.000461, -0.011647)`이다. 앞서 요청한
배포의 시작점 0 처리를 유지하므로, 두 관측에 동일하게 약 1.2cm의 원점 이동이
생긴다. 상대 위치 오차는 유지되지만 신경망 출력이 동일하다는 보장은 없다.
학습과 배포를 완전히 일치시키려면 학습에서도 같은 시작점 정규화를 적용해야 한다.

## 빌드와 실행

기존 실행 컨테이너에서 설정과 assets를 설치에 반영한다.

```bash
cd /root/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-up-to ai_sapiens_sim2real
source install/setup.bash
```

각 터미널에서 workspace를 source한 후, sim2sim은 기존 MuJoCo backend를 실행한다.

```bash
ros2 launch ai_sapiens_bringup k1_mujoco.launch.py
```

sim2real은 기존 `ros2 launch ai_sapiens_bringup k1.launch.py` backend를 사용한다.
별도 터미널에서 추정기와 정책을 시작한다.

```bash
ros2 launch ai_sapiens_sim2real glopodanamite.launch.py
```

이 wrapper는 공통 K1 설정을 사용하므로 Redred도 포함한다. 추정기를 이미
별도로 실행 중이면 `ai_sapiens_sim2real.launch.py robot:=k1`을 사용한다.
추정기의 초기 활성화와 키보드/DualSense 설정은
[기존 실행 절차](glopodanamite.md#실행)를 따른다. MuJoCo의 기본 gantry는
모션 재생 전에 viewer에서 기존 방식으로 지면 접촉 상태를 준비해야 한다.

- RadioMaster: **CH11 1120 → selector 206**, 기존 mimic 진입 조작 사용.
- 키보드: 좌우 방향키로 **MimicRedredGlopo (206)** 선택, `4`로 시작, `3`으로 Velocity.
- DualSense: selector 목록에서 **206** 선택 후 기존 mimic 버튼 사용.

assets는 Git ignore 대상이므로 다른 환경에는 `redred_glopo` 폴더도 복사한다.

## 검증

workspace를 source한 후 제공 ONNX와 합성 센서로 검사할 수 있다.

```bash
python3 ai_sapiens_sim2real/scripts/run_gloposition_smoke_test.py --redred
python3 ai_sapiens_sim2real/scripts/run_gloposition_smoke_test.py --redred --teleop dualsense
```

별도 localhost ROS domain에서 128개 관측, 모션 위치·속도, estimator 이동 반영,
보행·회전 후 재진입, localization timeout 시 Velocity 전환을 검사한다.
명령은 테스트 토픽으로만 발행한다. 실제 MuJoCo 물리 안정성과 하드웨어 춤 동작은
이 합성 센서 검사의 검증 범위에 포함되지 않는다.
