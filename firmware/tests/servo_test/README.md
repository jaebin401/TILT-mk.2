# PWM servo calibration test

Interactive ESP32-S3 test for finding a PWM servo's usable pulse range.

## Wiring

- Servo signal: GPIO 4 by default
- Servo power: a separate supply rated for the servo
- Servo ground and ESP32 ground: connected together

Do not power a loaded servo from the ESP32 3.3 V pin. Keep the horn or linkage
clear on the first run.

## Operation

The program boots with PWM output disabled. Open the ESP-IDF monitor and press
`o` to start at 1500 us. Use `a/d`, `s/w`, and `q/e` for 1, 10, and 100 us
adjustments. Press Space immediately if the servo reaches a mechanical stop,
stalls, chatters, or draws excessive current.

Record the smallest and largest pulse widths that remain safely clear of the
mechanical stops. The 500..2500 us software range is only an electrical guard
rail; it is not a safe travel guarantee for a particular servo or mechanism.

## Build and flash

```sh
cd firmware/tests/servo_test
idf.py build
idf.py flash monitor
```
