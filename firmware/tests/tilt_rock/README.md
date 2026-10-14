# TILT mk.2 — tilt_rock (rocking with ankle pitch)

`tests/dcm_rocking_test` 를 복제해 **8관절 다리**(hip roll / hip pitch / knee pitch / ankle pitch,
ID 11–14 · 21–24)에 맞춘 rocking 테스트. 키·절차·IMU 피드백·walk stride 는 dcm_rocking_test 와 같고,
아래만 다르다. 원본 README 의 결과·튜닝 기록은 `../dcm_rocking_test/README.md` 참고.

## 달라진 점

| | dcm_rocking_test (발목 고정) | tilt_rock |
|---|---|---|
| stand pose | `[0, -19, +40]` + 고정 브래킷 | `ZERO_POSE_RAD` = `[0, -20, +40, -20]` (세 pitch 관절 20°씩, 발바닥 평행) |
| 발바닥 평행 | 브래킷이 −20° 로 고정 | `hip + knee + ankle = 0` 을 유지 |
| lift (swing) | hip −L, knee +L | `V` 로 선택 (둘 다 발바닥 평행 유지) |
| CSV | 6관절 | 8관절 (`cmd_LAP`, `meas_LAP` …) |

### Lift mode (`V`)

| mode | 관절 변화 | stand 에서 발바닥 움직임 |
|---|---|---|
| `flash` (기본) | hip −L, knee +L, ankle 0 | 15°: 앞 +15.7 mm, 위 +8.2 mm — dcm_rocking_test 와 같은 다리 움직임 |
| `vertical` | hip −L, knee +2L, ankle −L | 15°: 위 +16.6 mm, 앞뒤 −0.3 mm — 허벅지·정강이가 대칭으로 접혀 수직 상승 |

기본값은 `flash`, 15° — 카펫에서 잘 되던 rocking 과 같은 다리 동작에서 시작해, 발목은 발바닥만 평행하게 잡는다.
`vertical` 은 같은 높이를 L 의 약 절반으로 낸다 (8° ≈ 7.8 mm). 처음엔 `-` 로 8–10° 로 낮춰 시작할 것.
IMU 피드백 layer 2 의 추가 lift 도 같은 mode 로 접힌다. `+`/`-` 범위는 4–22°.

## 컴포넌트 변경 (2026-10-13, tilt_config 8관절화)

- `tilt_kinematics`: 다리당 4관절 (`theta[NUM_LEG_JOINTS]`). URDF 에 아직 발목이 없어서,
  calf 위 `CALF_LENGTH_MM` 지점에 knee 축과 평행한 발목 축을 두고 URDF 의 발을 그 축으로 돌리는 모델.
  ankle = −20° 이면 예전 고정 발과 같아서 MuJoCo 정답과 0.001 mm 안에서 일치(host test T1).
  `ik_foot(leg, sole, sole_pitch = 0)` 은 발바닥 각도를 지정해서 푼다.
- `tilt_estimation`: 8관절 인덱스. 질량 모델은 아직 URDF 그대로 — **발목 서보 질량 미반영**.
- `tilt_dcm_gait`: `standPose()` = `ZERO_POSE_RAD`, `LiftMode` / `addLift()`, 피드백 trim 인덱스,
  `WalkStride` 는 발바닥 각도를 유지하는 IK.

## Before power-on

dcm_rocking_test 와 같음 (서보 전원 분리, 손 닿는 곳에 차단, 첫 실행은 잡거나 매달기). 추가로:

- `tilt_config.h` 의 `ZERO_TICK` / `JOINT_SIGN` / `JOINT_LIMIT` 이 servo_tool 로 확인된 상태일 것
  (`servo_tool_UART` 의 `h` HOME 에서 다리 일자 + 발바닥 평행, `u` STAND 에서 발바닥 평행).
- `withinLimits` 는 `JOINT_LIMIT` 밖 명령에서 E-STOP 한다.

## Build

```bash
source ~/esp/esp-idf/export.sh
cd firmware/tests/tilt_rock
idf.py set-target esp32s3
idf.py build flash monitor
```

## Procedure

dcm_rocking_test README 의 1–5 단계와 같다. 확인할 것:

1. **IMU** — `ImuMounting.h` 는 dcm_rocking_test 값을 그대로 가져왔다 (몸통 동일). stand 자세가 같으니
   `i` 로 확인만 하고, 바닥에 세워 `L` 레벨 보정을 한 번 다시 하는 것을 권장.
2. **Estimator (`e`)** — stand 에서 `h` ≈ 205–215 mm (발목 서보 질량이 빠져 있어 실제와 수 mm 차이 가능),
   `inner L/R` ≈ −36 mm.
3. **Single lift `r` / `l`** — 먼저 `flash`. 발이 뜨고 발바닥이 평행하게 내려오는지 본다.
   `T` 스텝 테스트는 lift mode 를 따른다 (vertical 이면 knee 목표 2L).
4. **`f` open loop** — `c` + 로그 켜고 `f`. dcm_rocking_test 와 같은 기본값
   (hold 150 ms / period 300 ms, IMU 피드백 layer 1·2 + pitch trim).
5. 잘 되면 `x` → `V` 로 `vertical`, `-` 로 lift 8–10° 에서 다시 `f`.

## 아직 남은 것

- URDF 에 발목 관절·서보 질량을 넣고 `RobotModel.h` 재생성 → 발목 모델·CoM 근사 제거.
- 발목 축 위치는 `CALF_LENGTH_MM`(knee→발목 69.4 mm), 발바닥까지 `FOOT_LENGTH_MM`(7 mm) 가정 — CAD 와 대조 필요.
- 발목으로 할 수 있는 것 (아직 안 씀): 착지 시 발바닥 각도 피드백, stance 중 ankle pitch 로 앞뒤 CoM 조절.
