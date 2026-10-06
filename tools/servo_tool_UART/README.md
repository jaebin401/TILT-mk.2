# TILT 서보 셋업 / 캘리브레이션 툴

`tilt_config.h` 의 TODO 항목을 실측으로 채우는 도구. 모든 결과는 **그대로 붙여넣을 수 있는 C 코드**로 출력된다.

| config 항목 | 채우는 툴 |
|---|---|
| `ZERO_TICK[]` | `j` 정밀 조그 → `z` |
| `JOINT_SIGN[]` | `s` 회전 부호 판별 |
| `JOINT_LIMIT[]` | `l` 가동범위 탐색 |
| `SERVO_UART_BAUD` | `b` 보레이트 스캔 |

## 위치

```
TILT/
├── firmware/
│   └── components/
│       ├── tilt_config/
│       └── tilt_sts3215/
└── tools/
    └── servo_tool_UART/     ← 이 프로젝트
```

루트 `CMakeLists.txt` 의 `EXTRA_COMPONENT_DIRS` 가
`../../firmware/components` 를 가리키므로, 이 디렉토리에서 바로 빌드하면
공용 컴포넌트가 딸려 온다.

## 빌드

```bash
cd tools/servo_tool_UART
idf.py set-target esp32s3
idf.py -p /dev/tty.usbmodemXXXX flash monitor
```

`Ctrl + ]` 로 모니터 종료.

## tilt_sts3215 연결 방식

```cpp
tilt::sts3215::Sts3215Bus bus{config};
bus.initialize();
bus.ping(id);
bus.readPosition(id, tick);
bus.writePosition(id, tick, speed);
bus.setTorque(id, enabled);
bus.setBaudRate(baud);
```

`main.cpp` 상단의 얇은 래퍼가 기존 도구 호출을 이 실제 API로 연결한다.

## 저장 정책

**진실 공급원은 `tilt_config.h` 하나뿐이다.**

NVS 는 캘리브레이션 도중 리셋·정전으로 작업이 날아가지 않게 하는 임시 스크래치일 뿐이다.
부팅 시 NVS 값이 있으면 그걸 쓰고 그 사실을 로그로 알린다.

캘리브레이션이 끝나면:
1. 출력된 배열을 `tilt_config.h` 에 붙여넣고
2. `c` 로 NVS 스크래치를 비운다

이 두 단계를 하지 않으면 "코드에 적힌 값"과 "로봇이 실제로 쓰는 값"이 갈라진다.

## 영점 잡는 순서

1. **`s` 부호 판별** — 먼저 해야 조그 화면의 DH theta 부호가 맞게 나온다
2. **`o` 엔코더 모니터** — 토크를 풀고 손으로 대략 영점 자세를 만든 뒤 `z`
3. **`j` 정밀 조그** — FINE(1 tick = 0.088deg)로 마무리, DH theta 가 0.00 에 수렴하면 `z`
4. **`0` zero pose 이동** — config 의 `[0, -20, +20]` 으로 이동해 검증
   - 발판이 지면과 평행한가
   - 접지점이 Hip Yaw 축 바로 아래(x=0)인가
   - 지면 ~ torso 원점 높이가 약 104mm 인가
5. **`l` 가동범위 탐색** — 마지막에. 영점이 확정돼야 각도 값이 의미를 가진다

### 각 관절의 DH theta = 0 자세

| 관절 | 영점 자세 |
|---|---|
| Hip Yaw | 발끝이 정면(+x) |
| Hip Pitch | 허벅지가 수직 |
| Knee Pitch | 정강이가 허벅지와 일직선 |

`zero pose` 인 `[0, -20, +20]` 과 혼동하지 말 것. 영점은 **DH theta = 0** 인 자세이고,
zero pose 는 거기서 허벅지를 20deg 앞으로 기울이고 무릎을 20deg 굽힌 **운용 기본자세**다.

### DH + 방향

| 관절 | + 방향 |
|---|---|
| Hip Yaw | 발끝이 왼쪽으로 |
| Hip Pitch | 다리가 뒤로 스윙 |
| Knee Pitch | 무릎이 굽음 |

좌우 다리 모두 같은 정의를 쓴다. 거울 반전은 `JOINT_SIGN` 이 흡수한다.

## 안전

- 서보 전원은 손 닿는 곳에 두고, 언제든 끌 수 있게 한다
- `space` = 전체 토크 OFF (어느 화면에서든)
- 부팅 시 절대 위치 명령을 보내지 않는다 — 현재 위치를 READ 해서 기준으로만 삼음
- `s` 부호 판별과 `0` zero pose 이동은 **공중에 매단 상태**에서 할 것
- `l` 가동범위 탐색은 다리끼리 부딪히기 직전까지만. 스탠스 폭이 72.4mm 라 Hip Yaw 안쪽 여유가 적다

## 툴 목록

| 키 | 기능 |
|---|---|
| `p` | ID 스캔 (0~253) |
| `b` | 보레이트 스캔 |
| `j` | 정밀 조그 (q/a w/s e/d = 왼쪽, r/f t/g y/h = 오른쪽, `m` 스텝 토글, `i` 상태표, `z` 영점 확정) |
| `o` | 토크 해제 + 엔코더 실시간 모니터 (`z` 로 영점 확정) |
| `l` | 가동범위 탐색 → `JOINT_LIMIT` 출력 |
| `s` | 회전 부호 판별 → `JOINT_SIGN` 출력 |
| `0` | zero pose 로 천천히 이동 |
| `v` | 현재 작업값 배열 전부 출력 |
| `c` | NVS 스크래치 비우기 |
| `t` | 전체 토크 ON/OFF 토글 |
| `h` | 메뉴 |
| `space` | 비상정지 |
