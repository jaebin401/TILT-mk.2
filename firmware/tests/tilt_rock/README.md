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
| `flash` | hip −L, knee +L, ankle 0 | 15°: 앞 +15.7 mm, 위 +8.2 mm — dcm_rocking_test 와 같은 다리 움직임 |
| `vertical` (기본) | hip −L, knee +2L, ankle −L | 15°: 위 +16.6 mm, 앞뒤 −0.3 mm — 허벅지·정강이가 대칭으로 접혀 수직 상승 |

동기 궤적 실측과 육안 검증을 마친 `vertical`이 현재 기본 mode다. 공중 시험에서 검증한 15°는
바닥 rocking에서 약 16.6 mm로 너무 높고 knee가 30° 접혀 roll 낙하를 키웠으므로, 기본 lift는
10°(약 10 mm)로 낮췄다. `vertical` 은 같은 높이를 L 의 약 절반으로 낸다 (8° ≈ 7.8 mm).
`V`를 누르면 기존 `flash` 동작과 서로 전환된다.
IMU 피드백 layer 2 의 추가 lift 도 같은 mode 로 접힌다. `+`/`-` 범위는 4–22°.

### 세 pitch 관절 동기 궤적

hip / knee / ankle pitch 에 최종 각도를 한 번에 쓰는 대신, 하나의 정규화된 5차 smoothstep 진행률을
세 관절 변화량에 곱해 10 ms 주기로 보낸다. 첫 실측에서 120 ms 명령에 대한 실제 `t90`이
170–210 ms로 늦게 나타나 상승/하강을 250 ms로 늘렸다. 정착 구간을 추가한 두 번째 실측에서는
실제 `t90`이 260–280 ms, 관절 간 편차가 20 ms였고 육안으로도 수직 상승이 확인됐다. 다음 rocking
시험 기본값은 상승/하강 200 ms다. 300 ms crossfade 바닥 시험에서는 내려오는 발이 실제 접지하기 전에
반대쪽 지지발을 올려 roll 방향으로 넘어졌기 때문에, 기본 동작은 완전 착지 후 다음 lift를 시작하는
순차 방식으로 되돌렸다. crossfade 구현은 비교 실험용 옵션으로만 남겨둔다.

- `vertical`의 매 주기 명령은 `hip = -L·s`, `knee = +2L·s`, `ankle = -L·s`다.
- 따라서 상승 중에도 `hip + knee + ankle` 합이 일정하고, 세 관절의 **명령 목표 도달 시각**은 같다.
- DCM 모드는 상승·하강 궤적이 끝나기 전에 다음 상태로 넘어가지 않는다.
- 기본 연속 open-loop는 한 발을 200 ms 동안 올리고 50 ms 유지한 뒤 200 ms 동안 내린다.
  이후 50 ms의 double-support 여유를 둔 다음 반대발을 올려 lift 시작 간격을 500 ms로 만든다.
- 단발 `r` / `l`도 마지막 발을 즉시 stand로 되돌리지 않고 200 ms 동안 내려놓은 뒤 종료한다.
- IMU feedback의 추가 lift도 같은 진행률을 사용하므로 궤적 중간에 별도 각도 step을 만들지 않는다.

서보의 실제 도달 시각은 부하와 서보 내부 제어에 따라 달라질 수 있다. 매달린 상태에서 `T`를 실행하면
현재 standing 출력 자세(학습된 pitch trim 포함)를 그대로 500 ms 유지해 안정화한 뒤 측정을 시작한다.
세 pitch 관절의 command/measured 표, 각 관절의 `t50`/`t90`, `t90 spread`, 기준 자세 대비 실제
발바닥 각도 오차(`sole_err`)가 출력된다. 이 안정화 단계는 STEP_TEST 진입 순간 pitch trim이 빠지면서
hip이 반대 방향으로 움직이는 측정 오염을 막는다.

## 컴포넌트 변경 (2026-10-13, tilt_config 8관절화)

- `tilt_kinematics`: 다리당 4관절 (`theta[NUM_LEG_JOINTS]`). URDF 에 아직 발목이 없어서,
  calf 위 `CALF_LENGTH_MM` 지점에 knee 축과 평행한 발목 축을 두고, 발바닥 접지 기준점은
  sole이 평평할 때 그 축 바로 아래 `FOOT_LENGTH_MM`에 둔다. 현재 실측값은 각각 70.431 mm와 22.9 mm다.
  URDF의 기존 fixed-foot 위치는 더 이상 sole 위치 계산에 사용하지 않는다.
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
2. **Estimator (`e`)** — 새 접지면 거리 반영 후 stand 에서 `h` ≈ 225–230 mm
   (발목 서보 질량이 빠져 있어 실제와 수 mm 차이 가능),
   `inner L/R` ≈ −36 mm.
3. **Single lift `r` / `l`** — 기본 `vertical`에서 발이 수직으로 뜨고 발바닥이 평행하게 내려오는지 본다.
   `T` 궤적 테스트는 lift mode 를 따르며 hip/knee/ankle의 실제 `t90`을 각각 출력한다
   (vertical 이면 knee 목표 2L, ankle 목표 −L).
4. **`f` open loop** — `c` + 로그 켜고 `f`. 바닥 시험용 기본값은 vertical 10°,
   trajectory 200/200 ms, single hold 250 ms, lift interval 500 ms, crossfade off다.
   각 발은 명령상 착지를 끝내고 50 ms가 지난 뒤 반대쪽 lift를 시작한다.
   첫 바닥 시험에서는 잡은 상태로 시작하고 흔들림이 커지면 즉시 `x`로 멈춘다.
5. 필요하면 `x` → `V`로 기존 `flash`와 비교하거나, `-`로 lift를 낮춰 다시 `f`.

## 아직 남은 것

- URDF 에 발목 관절·서보 질량을 넣고 `RobotModel.h` 재생성 → 발목 모델·CoM 근사 제거.
- 발목 축 위치 `CALF_LENGTH_MM`(knee→ankle 70.431 mm)와 접지면 거리
  `FOOT_LENGTH_MM`(ankle 축→foot pad 접지면 수직거리 22.9 mm)는 실측 반영됨.
- 발목으로 할 수 있는 것 (아직 안 씀): 착지 시 발바닥 각도 피드백, stance 중 ankle pitch 로 앞뒤 CoM 조절.
