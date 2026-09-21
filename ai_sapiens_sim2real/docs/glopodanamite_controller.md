# Glopodanamite controller: 춤을 추면서 이동·회전 조작

cyclo_mjlab의 `feature-k1-mimic-gloposition-controller` 브랜치,
`Cyclo-Mimic-K1-Rev1-Dynamite-Gloposition-controller` 정책을 실행한다.
기존 glopodanamite는 selector **203**, controller 버전은 **204**
(`MimicGlopodanamiteController`, behavior `mimic_glopodanamite_controller`)다.
최신 controller MoE는 별도 selector **205**로 등록했다.
[MoE sim2sim 문서](glopodanamite_controller_moe.md)를 참고한다.

## 학습 코드와의 연결

학습의 `source/tasks/mimic/mdp/steering.py`와 `commands.py`를 기준으로
동일한 명령 필터와 reference 보정을 C++ mimic runtime에 적용했다.
원본 춤 관절 궤적을 재생하면서, 조작 입력으로 추가 이동과 회전을 지시한다.
보행 정책의 절대 목표 속도와는 의미가 다르다.

| 항목 | 배포 처리 |
| --- | --- |
| Actor / action | 131 / 23차원, 제공된 ONNX 사용 |
| `[0:124]` | 기존 관절·IMU·이전 action·anchor 방향; anchor에는 steering yaw 반영 |
| `[124:126]` | estimator XY를 미믹 진입 좌표계로 변환 |
| `[126:128]` | CSV XY − 시작 CSV XY + 누적 steering offset |
| `[128:131]` | 필터를 거친 추가 `[vx, vy, yaw_rate]` |
| 주기 | 50 Hz, CSV 50 FPS (`fps * step_dt == 1` 검사) |
| 명령 범위 | vx/vy 각각 ±0.3 m/s, yaw rate ±0.3 rad/s |
| 필터 | 시간상수 0.5초, `alpha = 1 − exp(−0.02 / 0.5)` |
| 입력 | 기존 teleop; API 권한에서는 기존 `/cmd_vel` 경로 |

추가 vx/vy는 **현재 로봇의 heading 기준**이다. estimator 자세를 미믹 좌표계로
변환한 뒤 XY 명령을 회전시킨다. 양의 vy는 왼쪽, 양의 yaw rate는 좌회전이다.
원본 CSV의 한 프레임 이동량을 `dr`, 누적 회전을 `yaw`, 이번 회전량을 `dyaw`라고 하면:

```text
applied_velocity += alpha * (clamped_requested_velocity - applied_velocity)
v_add = R(current_robot_heading) * applied_velocity.xy
dyaw = applied_velocity.z * dt
offset += R(yaw + dyaw/2) * dr - dr + v_add * dt
yaw = wrap(yaw + dyaw)
reference_xy = csv_xy - csv_start_xy + offset
reference_orientation = yaw_quaternion(yaw) * csv_reference_orientation
```

회전 명령은 춤의 원래 이동 경로도 연속적으로 회전시킨다. 명령을 0으로 하면
추가 속도가 점차 줄어들며, 이미 누적된 위치·방향 보정은 유지된다.
원본 CSV를 변경하거나 춤 시작 위치로 되돌리지 않는다. 학습의 무작위 명령
샘플링은 배포에서 사용하지 않는다.

보행 후 미믹에 다시 진입할 때마다 estimator XY/yaw 기준과 steering 누적값을
새로 잡는다. **첫 obs는 로봇 XY, reference XY, applied velocity 모두 0**이다.
조이스틱을 잡고 진입해도 그 다음 프레임부터 속도가 올라간다. estimator는
세션 동안 유지한다. localization 누락·정지 시 기존과 동일하게 미믹을 종료하고
Velocity로 전환한다.

이전 요청에 따른 시작 XY 정규화를 유지했다. 기존 204 export의 학습 XY obs는
CSV 시작점을 빼지 않으므로 기존 ONNX와 완전히 같은 절대 좌표 분포는 아니다.
이는 기존 203/204 모델에 공통이며 [기존 문서의 학습 좌표 차이](glopodanamite.md#학습-좌표와의-차이)를
참고한다. 새 205 MoE export는 학습에서도 episode 원점을 사용한다.

## 파일과 실행

```text
assets/k1/mimic/glopodanamite_controller/
├── exported/policy.onnx
└── params/
    ├── sim2real.yaml
    └── dynamite004_headwrap_v3.csv
```

`config/k1_config.yaml`은 다른 미믹과 같은 `kind`, `asset`, `motion` 형식을 사용한다.
명령 범위와 시간상수는 해당 asset의 `params/sim2real.yaml`에서 읽는다.
제공한 YAML의 빈 `commands: {}`에 아래 설정을 추가했다.

```yaml
commands:
  reference_trajectory:
    steering:
      lin_vel_x: [-0.3, 0.3]
      lin_vel_y: [-0.3, 0.3]
      yaw_rate: [-0.3, 0.3]
      smoothing_time_constant: 0.5
```

YAML을 재수출하여 교체할 때에도 이 설정을 포함해야 한다. global XY와 velocity obs가 있는데 steering 설정이 빠지면
시작 시 오류로 알린다. 다른 정책의 보행 범위를 가져오지 않는다.
로컬 asset YAML에는 학습 runner의 `clip_actions: 10.0`에 맞춰
`actions.joint_pos.raw_clip`을 23개 `[-10, 10]`으로 추가했다.
**assets는 Git ignore 대상**이므로 다른 실행 환경에도 이 폴더를 복사하고 재빌드한다.

빌드와 estimator 활성화 절차는 [glopodanamite 실행 문서](glopodanamite.md#빌드)와 같다.
두 환경 모두 같은 policy runtime을 사용하고 backend만 선택한다.

```bash
# sim2sim backend
ros2 launch ai_sapiens_bringup k1_mujoco.launch.py
```

```bash
# sim2real backend
ros2 launch ai_sapiens_bringup k1.launch.py
```

추정기가 이미 실행 중이라면 기존 policy launch를 사용한다.

```bash
ros2 launch ai_sapiens_sim2real ai_sapiens_sim2real.launch.py robot:=k1
```

추정기와 policy를 함께 시작할 때는 기존 `glopodanamite.launch.py`를 사용한다.
이 wrapper는 glopodanamite 계열 상태를 모두 포함하는 K1 설정을 로드한다.
추정기 초기 활성화·bias 수집은 기존 절차대로 세션 시작 시 한 번 진행한다.

RadioMaster는 CH11 **1080 → 204** 선택 후 기존 mimic 진입 조작을 사용한다.
키보드/DualSense의 selector 목록에도 새 상태가 자동으로 나타난다.
키보드는 좌우 방향키로 204 선택, `4`로 진입, `w/s` 전후, `a/d` 좌우,
`q/e` 회전, 스페이스로 추가 명령을 0으로 만든다. 키보드 명령은 누적되므로
키에서 손을 떼어도 유지되고 **스페이스를 눌러야 0**이 된다.
`3`을 누르면 미믹을 종료하고 Velocity로 전환한다.

## 검증

```bash
python3 ai_sapiens_sim2real/scripts/run_gloposition_smoke_test.py --controller
```

실제 제공 ONNX와 합성 센서로 131차원 obs, 프레임별 XY/방향 보정 수식,
명령 필터·release, 보행·회전 후 재진입, localization timeout의 Velocity 전환을 검사한다.
별도 localhost ROS domain에서 명령을 테스트 토픽으로만 발행한다.
기존 128차원 정책은 같은 스크립트에서 `--controller`를 생략하여 검사한다.
단위 테스트에는 중간 yaw 적분, 현재 heading 기준 이동, 범위 제한, 누적 보정 유지,
초기화 및 잘못된 설정 거부를 포함한다. 실제 MuJoCo 물리 안정성이나 하드웨어
동작 성공을 검증하는 테스트는 아니다.

### DualSense가 움직임에 반영되지 않을 때

설치된 정책 노드와 DualSense 플러그인 경로를 합성 입력으로 검사할 수 있다.
이 테스트는 실제 패드나 로봇을 조작하지 않는다.

```bash
python3 ai_sapiens_sim2real/scripts/run_gloposition_smoke_test.py --controller --teleop dualsense
```

실제 실행에서는 기존 policy launch에 `debug_publish_enabled:=true`를 추가하고,
별도 터미널에서 아래 읽기 전용 도구를 실행한다. 실행 중인 노드의
`debug_publish_enabled` 값을 `ros2 param set`으로 변경하는 방식은 지원하지 않는다.

```bash
python3 /root/ros2_ws/src/ai_sapiens/ai_sapiens_sim2real/scripts/inspect_mimic_steering.py
```

활성 상태·권한, 패드 raw 축, 실제 obs 마지막 3개, 로봇·reference XY를 함께 표시한다.
`joy`가 변해도 `applied`가 0이면 권한과 입력 경로를 확인한다. `applied`와
reference는 변하는데 로봇이 따라가지 못하면 정책 추종과 시뮬레이터 고정 상태 등을
확인한다. 이 도구는 원인 판단에 필요한 값을 보여주며 로봇 명령을 발행하지 않는다.
