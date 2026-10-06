# STS3215 Servo Setup Tool

맥북에서 Waveshare Bus Servo Adapter (A)를 통해 STS3215-C001 서보의 통신 상태와 현재 위치를 확인하고, 혼을 조립하기 위한 기계적 중앙 위치 `2047`로 이동시키는 도구입니다.

이 도구는 ESP32 펌웨어가 아닙니다. 맥에서 직접 실행하는 일회성 설정 도구이며, 서보 검색, ID 변경, 기계적 중앙 정렬을 지원합니다. 영구 오프셋 보정은 아직 포함하지 않습니다.

## Safety

- STS3215-C001에는 안정화된 7.4V 전원을 사용합니다. 12V를 연결하지 않습니다.
- Waveshare 보드는 입력 전압을 조절하지 않으므로 입력 전압이 그대로 서보에 전달됩니다.
- USB 제어 시 Waveshare 점퍼를 `B` 위치에 둡니다.
- 처음에는 서보 한 개만 연결합니다.
- `set-id` 명령은 반드시 서보 한 개만 연결한 상태에서 실행합니다. 같은 ID의 서보 여러 개를 동시에 연결하지 않습니다.
- `center` 명령 전에 혼과 링크를 분리합니다.
- 외부 서보 전원을 즉시 끌 수 있는 상태에서 실행합니다.
- 서보가 움직이는 동안 축과 링크를 손으로 잡지 않습니다.

## Environment Setup

```bash
cd tools/servo_setup

python3 -m venv .venv
source .venv/bin/activate

python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

## Commands

### List serial ports

```bash
python servo_setup.py list-ports
```

Waveshare 보드는 일반적으로 다음과 비슷한 이름으로 나타납니다.

```text
/dev/cu.wchusbserialXXXX
/dev/cu.usbserial-XXXX
```

### Ping one servo

공장 초기 ID는 일반적으로 `1`이고 기본 통신 속도는 1,000,000 baud입니다.

```bash
python servo_setup.py ping \
  --port /dev/cu.wchusbserialXXXX \
  --id 1
```

### Find the current servo ID

서보 ID를 모르거나 `ping`에 응답하지 않으면 유효한 ID 범위 `0~253` 전체를 검색합니다. 이 명령은 서보를 움직이거나 설정을 변경하지 않습니다.

```bash
python servo_setup.py check-id \
  --port /dev/cu.wchusbserialXXXX
```

검색은 기본 통신 속도인 1,000,000 baud에서 실행되며 응답이 없는 경우 약 15초가 걸릴 수 있습니다. 다른 통신 속도를 확인하려면 `--baud`를 지정합니다.

```bash
python servo_setup.py check-id \
  --port /dev/cu.wchusbserialXXXX \
  --baud 500000
```

서보가 발견되면 ID와 모델 번호가 출력됩니다. 발견된 ID는 이후 `ping`, `read`, `center`, `release` 명령의 `--id`에 사용합니다.

### Change one servo ID

ID 변경은 EEPROM에 저장되므로 전원을 껐다 켜도 유지됩니다. 새 서보는 일반적으로 ID `1`이므로, 반드시 대상 서보 한 개만 연결한 뒤 실행합니다.

```bash
python servo_setup.py set-id \
  --port /dev/cu.wchusbserialXXXX \
  --current-id 1 \
  --new-id 11
```

도구는 다음 순서로 안전하게 변경합니다.

1. 기존 ID가 응답하는지 확인합니다.
2. 새 ID가 이미 사용 중인지 확인합니다.
3. `SET 1 11`과 같은 확인 문구를 요구합니다.
4. 토크를 해제하고 EEPROM 잠금을 해제합니다.
5. ID를 기록하고 새 ID로 EEPROM을 다시 잠급니다.
6. 새 ID는 응답하고 기존 ID는 응답하지 않는지 검증합니다.

성공하면 전원을 끄고 모터와 케이블에 관절 약어와 ID를 함께 표시합니다. 그다음 서보로 넘어갈 때는 새 서보만 연결하고 같은 과정을 반복합니다.

### Read the current position

```bash
python servo_setup.py read \
  --port /dev/cu.wchusbserialXXXX \
  --id 1
```

출력되는 각도는 중앙 위치 `2047`을 `0°`로 환산한 참고값입니다.

### Move to the mechanical center

```bash
python servo_setup.py center \
  --port /dev/cu.wchusbserialXXXX \
  --id 1
```

도구가 현재 위치와 예상 이동량을 출력합니다. 혼과 링크가 제거된 것을 다시 확인한 뒤 `CENTER`를 입력해야 실제 명령이 전송됩니다.

서보가 `2047 ± 10`에 도착하면 외부 서보 전원을 끄고 축을 움직이지 않도록 주의하면서 혼을 관절의 기계적 중립 위치에 체결합니다.

`center` 명령은 성공 후 토크를 유지합니다. 혼을 장착하기 전에 반드시 외부 서보 전원을 끕니다. 이동 중 오류가 발생하거나 `Ctrl+C`로 중단하면 도구가 토크 해제를 시도합니다.

### Release torque

```bash
python servo_setup.py release \
  --port /dev/cu.wchusbserialXXXX \
  --id 1
```

토크가 해제된 축은 손으로 움직일 수 있습니다. 토크가 활성화된 상태에서 축을 억지로 움직이지 않습니다.

## Joint Structure

각 다리는 몸통에서 발 방향으로 다음 관절 순서를 사용합니다.

```text
Hip Yaw → Hip Pitch → Knee Pitch
```

좌우는 관찰자가 아니라 로봇이 바라보는 방향을 기준으로 합니다. 권장 ID는 십의 자리로 다리를, 일의 자리로 관절 순서를 구분합니다.

| Label | ID | Joint |
| --- | ---: | --- |
| `LHY-11` | 11 | Left Hip Yaw |
| `LHP-12` | 12 | Left Hip Pitch |
| `LKP-13` | 13 | Left Knee Pitch |
| `RHY-21` | 21 | Right Hip Yaw |
| `RHP-22` | 22 | Right Hip Pitch |
| `RKP-23` | 23 | Right Knee Pitch |

새 서보 여섯 개가 모두 ID `1`이라면 각 모터를 하나씩 연결해 다음 순서로 할당합니다.

```text
1 -> 11  Left Hip Yaw
1 -> 12  Left Hip Pitch
1 -> 13  Left Knee Pitch
1 -> 21  Right Hip Yaw
1 -> 22  Right Hip Pitch
1 -> 23  Right Knee Pitch
```

모든 할당이 끝난 뒤 여섯 개를 함께 연결하고 최종 확인합니다.

```bash
python servo_setup.py check-id \
  --port /dev/cu.wchusbserialXXXX
```

최종 검색 결과에는 `11, 12, 13, 21, 22, 23`이 각각 한 번씩 나타나야 합니다.

## Mechanical Center vs Offset Calibration

현재 도구가 수행하는 작업은 서보를 raw position `2047`로 이동시킨 뒤 혼을 기계적 중립 위치에 체결하는 것입니다.

현재 축 위치를 새로운 중앙으로 저장하는 전자적 오프셋 보정은 수행하지 않습니다. 오프셋 보정은 전체 골격 조립 후 스플라인 간격으로 해결할 수 없는 오차가 있을 때 별도 절차로 진행합니다.
