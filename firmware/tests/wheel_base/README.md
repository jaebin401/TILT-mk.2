# TILT mk.2 — wheel_base

ESP32-S3에서 STS3215 다리 관절 8개와 TB6612FNG의 DC 모터 2개를 함께
제어하는 테스트 펌웨어다. 보행은 하지 않는다. HOME 또는 STAND를 기준으로
몸통의 sagittal `x/z` 위치와 양쪽 hip pitch를 조절하며, knee/ankle IK가 발
접촉점 위치를 맞춘다. 발바닥 각도는 구속하지 않는다.

## 기본 배선

최종 캐리어 보드와 충돌하면
`firmware/components/tilt_config/include/tilt/config/HardwareConfig.h`에서
GPIO를 변경한다.

| ESP32-S3 | TB6612FNG | 역할 |
|---|---|---|
| GPIO4 | PWMA | 왼쪽 모터 MCPWM |
| GPIO5 | AIN1 | 왼쪽 방향 1 |
| GPIO6 | AIN2 | 왼쪽 방향 2 |
| GPIO7 | PWMB | 오른쪽 모터 MCPWM |
| GPIO10 | BIN1 | 오른쪽 방향 1 |
| GPIO11 | BIN2 | 오른쪽 방향 2 |
| GPIO12 | STBY | 소프트웨어 enable / 긴급정지 |
| 3.3V | VCC | 드라이버 로직 전원 |
| GND | GND | ESP32·TB6612·모터 전원 공통 GND |
| 외부 모터 전원 + | VM | 모터 정격 전압에 맞는 전원 |

STS3215는 UART1 GPIO17(TX), GPIO18(RX), 1 Mbps를 사용한다. 모터 전원을
ESP32의 3.3V에서 공급하지 않는다.

## 키 조작

| 키 | 동작 |
|---|---|
| 방향키 위 / 아래 | 전진 / 후진 |
| 방향키 왼쪽 / 오른쪽 | 제자리 좌회전 / 우회전 |
| `+` / `-` | 목표 속도 10% 증감 |
| `x` | 가감속 제한을 적용해 정지 |
| `o` | 현재 관절 위치에서 arm |
| `h` / `s` | HOME / STAND 이동 후 IK 기준 캡처 |
| `i` / `k` | 몸통 위 / 아래 2 mm |
| `j` / `l` | 몸통 뒤 / 앞 2 mm |
| `q` / `e` | 양쪽 hip pitch −/+ 0.5° |
| `r` | 현재 HOME/STAND 기준 몸통 목표로 복귀 |
| `p` | 상태 출력 |
| `Space` | 모터 STBY LOW 및 서보 토크 OFF 긴급정지 |

방향키 명령은 터미널이 key-up 이벤트를 보내지 않으므로 다른 방향키, `x`,
또는 `Space`를 누를 때까지 유지된다. 정상 정지는 jerk/가속도 제한을 적용하고,
긴급정지만 즉시 출력을 차단한다.

HOME은 다리가 거의 완전히 펴지는 특이점이다. HOME 기준 IK는 지원하지만
일부 이동 방향은 기구학적으로 불가능하므로 매달린 상태에서 작은 명령부터
시험한다. 도달 불가능하거나 관절 제한을 넘는 명령은 취소되고 마지막 정상
명령을 유지한다.

## 실행 순서

1. 바퀴를 바닥에서 띄우고 모터 전원을 끈 채 부팅한다.
2. 모든 STS3215의 ping과 startup torque OFF를 확인한다.
3. `o`로 arm한다.
4. `s`로 STAND에 도달한 뒤 `i/k/j/l/q/e`를 작은 범위에서 확인한다.
5. 모터 전원을 켜고 방향키를 짧게 시험한다. 방향이 반대라면 모터 선을
   바꾸거나 `kWheelLeftInverted` / `kWheelRightInverted`를 변경한다.
6. `x`의 감속 정지와 `Space`의 즉시 긴급정지를 확인한 뒤 바닥 시험을 한다.

## 빌드

```bash
source /Users/jae/.espressif/tools/activate_idf_v6.0.1.sh
export PATH="$IDF_PATH/tools:$PATH"
cd "firmware/tests/wheel_base"
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/cu.usbmodemXXXX flash monitor
```
