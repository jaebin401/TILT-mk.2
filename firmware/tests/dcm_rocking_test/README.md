# TILT mk.2 DCM rocking test

Stepping in place (lateral rocking) gated by the estimated DCM.

- `tilt_estimation::DcmEstimator` — CoM / DCM from servo read-back + MPU6050 + URDF mass model
- `tilt_dcm_gait::DcmStepping` — FLASH lifts (swing hip −lift, knee +lift at once, sole parallel)
  - **DCM mode `d`**: first lift open loop (90 ms) → touch down when the DCM comes back to within
    `touchdown_inside` (21 mm) of the swing foot centre → lift the other leg when the DCM passes the
    new stance foot's inner edge (`lift_past_inner`, 0 mm)
  - **Open-loop mode `f`**: FLASH baseline, hold 90 ms / period 200 ms
- Stops: step limit (default 10), double-support timeout (300 ms), DCM beyond the stance foot's outer
  edge (+15 mm), IMU tilt > 20° (gait stop) and > 35° (E-STOP, torque off)

Uses the existing components `tilt_config`, `tilt_sts3215`, `tilt_MPU6050` unchanged.

## Before power-on

- Servo power separate from the ESP32, shared GND, a physical power cut within reach.
  SPACE cannot help if USB or servo power fails.
- Arms and head at their zero pose: the CoM model assumes it.
- First runs: one hand ready to catch, or a loose overhead tether.

## Build

```bash
source ~/esp/esp-idf/export.sh
cd firmware/tests/dcm_rocking_test
idf.py set-target esp32s3
idf.py build flash monitor
```

In `idf.py monitor`, **Ctrl+T then Ctrl+L** starts/stops saving the output to a log file.

## Keys

| Key | Action |
|---|---|
| `o` / `s` / SPACE | arm at measured pose / stand (2 s) / E-STOP |
| `i` / `e` / `c` | IMU monitor / estimator monitor (10 Hz) / CSV every cycle |
| `X` `Y` `Z` / `W` / `k` | flip IMU axis sign / swap IMU x,y / confirm IMU mapping |
| `b` / `p` / `v` | servo read-back on/off / status / params |
| `r` / `l` | one open-loop lift, right / left leg |
| `f` / `d` / `x` | open-loop stepping / DCM stepping / stop → stand |
| `+` `-` | lift ±1° (8–22°) |
| `[` `]` | touchdown_inside ∓1 mm |
| `;` `'` | lift_past_inner ∓1 mm |
| `,` `.` | open-loop period ∓10 ms |
| `n` `m` | max_steps ∓2 (0 = unlimited) |

## Procedure

### 1. IMU check (servos can stay off)

DCM stepping is locked until the IMU mapping is confirmed. Press `i` and hold the robot:

| Pose | Expected (robot frame) | If not |
|---|---|---|
| upright, level | acc z ≈ +1.0 g, x ≈ y ≈ 0 | z ≈ −1 → `Z` |
| nose down ~30° | acc **x** ≈ −0.5, pitch **+** | y changes instead of x → `W`; x positive → `X` |
| robot's **left side up** ~30° (leans to its right) | acc **y** ≈ +0.5, roll **+** | y negative → `Y` |
| while lifting the left side | gyro x **+** | — (follows the same flips) |
| while tipping nose down | gyro y **+** | — |

A physical mounting is a rotation, so if you had to flip `Z` exactly one of `X`/`Y` should also be
flipped. Press `k`, then copy the printed values into `main/ImuMounting.h` and set
`kImuVerified = true` so the next boot starts unlocked.

### 2. Estimator check (standing)

`o` (hold the robot), `s`, put it on the floor, `e`:

- `h` ≈ 205–215 mm, `w` ≈ 6.8, `com y` within ±5 mm, `vy` near 0
- `inner L` and `inner R` both ≈ −36 mm (the DCM is between the feet)
- Push the torso gently to the robot's **left** so it rocks onto the left foot: `roll` goes
  negative and `inner L` rises toward/above 0. If it is the other foot, the IMU mapping is wrong.
- `p`: `readback` time should be a few ms and `overruns` should not keep increasing.
  If they do, `b` turns read-back off (the estimator then uses commanded angles + a lag model,
  ~6 mm less accurate in simulation).

### 3. Single lifts

`r`, then `l`. One lift each (90 ms). Check that the foot leaves the floor and lands cleanly.

### 4. Open-loop baseline

`c` (CSV on), Ctrl+T Ctrl+L (log on), `f`. 10 steps by default. Keep this log to compare with DCM mode.

### 5. DCM stepping

`d`. Each phase change prints an `EVT` line with its trigger; the run ends with a summary:

```
== run: DCM, 10 steps in 2.1 s, stop=step_limit | swings 10 (dcm touchdown 9, timeout 0) ...
```

| Symptom | Adjust |
|---|---|
| many `swing_timeout` | DCM never comes back far enough → raise `touchdown_inside` (`]`) |
| stop `double_timeout` | DCM does not pass the new stance foot's inner edge → lower `lift_past_inner` (`;`, e.g. −5 mm) |
| stop `outer_edge` | falling outward over the stance foot → land earlier (`]`) or lift less (`-`) |
| foot does not clear the floor | `+` |

**Pass:** 20 steps (`m` to raise the limit) with most swings ending on `dcm touchdown`, no catch.

## Logs

```bash
python3 tools/dcm_log/extract.py path/to/monitor.log
```

Writes `monitor.csv`, `monitor_events.txt` and `monitor.png` (DCM edge margins with swing phases,
roll, cycle time).
