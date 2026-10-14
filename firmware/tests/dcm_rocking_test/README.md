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

## Results — 2026-10-08 (first hardware runs)

Settings (now the boot defaults of this app): servo acceleration limit reg85 = reg41 = 254 (written to
servo RAM at boot), lift 15°, first lift 150 ms, swing 100–240 ms, `touchdown_inside` 16 mm,
`lift_past_inner` 0 mm, double support 20–300 ms, servo read-back on, IMU map x−1 y−1 z+1 with the
level calibration in `ImuMounting.h`.

| | slippery floor | carpet |
|---|---|---|
| 10 steps | 1.6 s | 3.1 s (≈ 0.31 s per step) |
| roll amplitude | ±4–5° | **±11–14°** |
| swing end | DCM touchdown (9/10), 120–160 ms | swing_max 240 ms (8/10) |
| knee fold reached | 5–9° | 15° (full) |
| DCM over stance foot (inner edge margin) | rarely | +20–25 mm every swing |

On carpet the robot rocks onto the stance foot during each swing (the swing side is lifted by
~12° of roll on top of the fold), the touchdown happens at `swing_max` near the roll apex, and the
DCM lift gate sets the double-support time (40–200 ms) so the rhythm locks to the body's own
rocking. On the slippery floor the stance foot slides, the body does not pivot over the foot edge,
the DCM swings back toward the swing foot and the touchdown rule lands it early.

Notes: left swings rock less than right swings (+6…+11° vs −12…−14°) and the left amplitude grew over
the run; check 20+ step runs before raising `max_steps` further. Sole friction matters more than the
gains: add rubber/TPU soles before tuning on smooth floors.

`results/2026-10-08_floor_compare.png`: roll, DCM edge margins and knee fold, same settings on both floors.

## IMU feedback layers (2026-10-09)

`components/tilt_dcm_gait/GaitFeedback.{h,cpp}` adds three IMU feedback layers on top of the stepping
pattern (both `f` and `d`). The servos get `pattern + feedback`; the CSV `cmd_*` columns show that sum.

| layer | rate | measures | acts on | goal |
|---|---|---|---|---|
| 1 trim | per gait cycle (2 swings) | cycle-mean roll / pitch | common hip roll / hip pitch offset | mean roll 0°, mean pitch `pitch_target` (2.5°) |
| 2 amplitude | per swing | peak roll toward the stance side after that swing | that side's extra lift (±3°) | excursion `amp_target` (7°) both sides |
| 3 damping | every 10 ms | low-pass gyro roll / pitch rate | common hip roll / pitch offset (±3°) | damp fast tipping (gain 0 = off) |
| ramp | first swings | – | lift ×1/4, 2/4, 3/4 | soft start |

Boot defaults: layer 1 roll trim ON, pitch trim **off** (sign not yet verified on hardware), layer 2 ON,
layer 3 off (pitch gain 0.03, roll gain 0), start ramp **off**: with the 3-step ramp the robot
fell at step 9 in two runs (trims still 0) — the small first lifts start the rocking out of phase with
the 300 ms timer, and by step 8 the right leg lifts while the body leans onto it.
Learned values (trim, lift deltas) are kept between runs and in STANDING; `0` clears them.
Layers 1 and 2 do not learn during the ramp or from a window whose |roll| or |pitch| exceeded 12°;
layer 1 does not learn from a cycle whose mean is more than 5° off. Layer 2 measures the excursion from
the previous cycle mean. Limits: roll trim ±4°, pitch trim ±5°, ≤0.5° per cycle, applied at ≤10°/s.
A trim stuck at its limit while the error stays above 1° prints `!ROLL TRIM SATURATED`.

Signs: hip roll axis +x and hip pitch axis +y on both legs; the feet keep their orientation on the floor,
so a common joint change dq rotates the torso by −dq. Trim dq = +gain·(measured − target).
Hardware run 2026-10-09 (first version had the roll sign reversed): +roll trim drove the mean roll from
−0.8° to −11°, i.e. the CoM-shift model was wrong and the torso-rotation model holds.

Keys (work while stepping): `1` roll trim, `5` pitch trim, `2` amplitude, `3` damping, `4` ramp (toggles), `g` state,
`0` reset learned values, `u`/`j` pitch target ±0.5°, `t`/`y` amplitude target ±0.5°,
`q`/`a` damping pitch gain ±0.01, `Q`/`z` damping roll gain ±0.005.

Each completed cycle prints a line:
`FB trim roll … pitch … | lift delta L … R … | last excursion L … R … | cycle mean roll … pitch …`.
CSV adds `fb_roll_trim, fb_pitch_trim, fb_lift_L, fb_lift_R, fb_damp_roll, fb_damp_pitch`.

Test plan (log with Ctrl+T Ctrl+L, `c` before each run):
1. Flat floor, `1` and `2` off (`g` to check), `f` ~20 s → baseline. Then both on, `0`, `f` ~20 s.
   Feedback must not make it worse; excursions should sit near 7° on both sides.
2. Slope (book under one side, ~2°): layers off vs on. Expect the cycle-mean roll to drop to ~0 within
   ~5 cycles and the left/right excursions to even out. **If the mean roll grows instead and the trim
   runs to ±4°, the sign is wrong: stop (`x`) and send the log.**
3. Same slope turned 90° (front/back) for the pitch trim.
4. Only if 1–3 still wobble: `3` on with the pitch gain 0.03, then 0.05.

### Feedback results — 2026-10-09 (flat floor, lift 15°, ramp off)

| run | layers | result |
|---|---|---|
| baseline | all off | 80 steps / 24 s, three disturbed stretches, fell (lifted the loaded leg at roll +6.4°) |
| A | roll trim + amplitude | 104 steps / 31 s, operator stop, max \|roll\| 10.3° |
| A repeat | same | 96 steps / 29 s, operator stop, max \|roll\| 9.2° |

In both feedback runs the first ~10 s were the learning phase, then the rhythm locked: excursions
5.3 ± 0.3° on both sides, touchdown roll ±4.0–4.5°, cycle-mean roll within ±0.4°, roll trim settling at
−1.1…−1.9°. The amplitude layer saturated at +3° on both sides (effective lift 18°) while the excursion
stayed at ~5.3° (< 7° target), i.e. layer 2 currently acts as a slow lift increase 15° → 18°.
Starting a run directly with lift 18° pitched the robot forward (cycle-mean pitch 3 → 11°) and it fell
within 1.6 s, so learned lift deltas are now faded in over `lift_fade_cycles` = 8 cycles per run.
The mean pitch creeps from 2.5° to 3.5° at the higher lift; pitch trim (`5`) is the next test.

Pitch trim (`5`) on hardware: the trim went to +0.6…+0.95° and held the cycle-mean pitch at ≈2.5°
(1.8–3.2° vs 2.7–3.9° without it) — sign confirmed.

### Layer 0: phase sync (`6`, off by default)

Remaining failure mode (baseline steps 36/54/80, feedback run step 24): the open-loop timer lifts a leg
at the wrong point of the rocking cycle. In steady rocking the lift happens at a repeatable phase (the
body has peaked toward that side and is already rolling back), so layer 0 measures the rocking phase
`theta = atan2(-rate/omega, lean)` toward the next swing side (lean from the cycle mean, omega =
2π/0.6 s), learns the lift phase per side from 8 lifts after cycle 10, and from then on lifts when the
body reaches that phase — at most `lift_hint_window_ms` = 60 ms before or after the timer. In a locked
rhythm the lifts stay on the timer; EVT lines show the phase at each lift (`ph=`) and the trigger
`phase_sync`; the run summary counts phase-sync lifts. `0` also clears the learned phases.

First hardware run with phase sync (±60 ms, 2026-10-09): 185 steps / 57 s, operator stop (longest run so
far), max |roll| 14.1°. The lift interval stretched to 305–313 ms on average (the body's own period is
~610–625 ms), but the rhythm was less regular than without sync (interval ±32–45 ms, excursion ±1.2–2.7°)
because the phases were learned during a period-2 rhythm. Changes: learning now waits for a settled
rhythm (`stable` in the FB line: last 4 excursions per side within 2°, cycle-mean roll within 1°), and
the window is 40 ms by default (`w` cycles 20/30/40/60 ms). No continuous re-learning: lifts triggered at
the nominal phase would bias it upward by up to one control tick (~6°) per lift.

A/B, 2026-10-09 evening (flat floor, layers 1+2+pitch trim, `0` before each run):

| | phase sync OFF (A) | phase sync ON, ±40 ms, learn gate (B) |
|---|---|---|
| runs | 65.6 s, 65.4 s (operator stop); one fall at 4.2 s during start-up | 60.6 s (operator stop) |
| lift interval after 10 s | 300 ± 0 ms | 305–311 ± 25–29 ms |
| excursion L / R after 20 s | 5.7 ± 0.3 / 5.4 ± 0.5° | 5.7 ± 1.9 / 5.7 ± 1.9° |
| \|cycle-mean roll\| | 0.23° | 0.9–1.0° |

Phase sync keeps the robot up but makes the rhythm noisier (each moved lift perturbs the next cycle);
the open-loop timer plus layers 1–2 gives a clean, repeatable limit cycle. Defaults: phase sync off,
pitch trim on. The remaining risk is the first ~5 s of a run (falls at 2.6 s, 4.2 s, 7.5 s before the
trims/lift had settled).

## Walking: fore-aft stride (2026-10-09)

`components/tilt_dcm_gait/WalkStride.{h,cpp}` shifts each sole fore-aft in the torso frame (leg IK,
added as a delta from the stand pose on top of the lift fold and the feedback layers):
swing x = +s/2 (stepped with the lift), stance x = +s/2 → −s/2 linearly over the stance time
(2·period − hold = 450 ms). The stride s ramps to its target at 4 mm/s, so every run starts in place.
At stride s the torso moves ≈ s / 0.45 s (10 mm → ≈ 22 mm/s). Joint size: +5 mm ≈ −2.1° hip pitch.

Why the robot drifted forward while "rocking in place": the FLASH fold (hip −15°, knee +15°) lifts the
sole only ~8 mm but moves it ~16 mm **forward** (19 mm at 18°); it comes back while landing, which
works like a short step. A small negative stride (`B`) cancels it.

Keys: `F` / `B` stride target ±2 mm (live). `x` while walking ramps the stride to 0 and then stops in
double support; `x` again stops at once. CSV adds `walk_stride, foot_x_L, foot_x_R`; FB lines show
`walk now/target`.
No turning yet (no hip yaw); only straight forward / backward.

Walking runs 2026-10-09 (flat floor): an 89 s run held strides −4…+4 mm with the same clean rhythm as
in-place rocking (excursions 5–6°, mean roll within ±0.6°) and fell ~5 s after +6 mm; an earlier run
held up to ~+10 mm while the stride was still ramping. Starting a run with a stride already set lost
balance at once, so every run now starts at `stride_in_place_mm` = −2 mm (rocks in place on this
floor) and `F` / `B` change it afterwards.

Evening start-up falls (2026-10-09, ~19:30): every fall came in the first ~5 s with the cycle-mean pitch
jumping to +5…+16° (forward). Two causes in the code: (1) a run that ended in a fall left its last
learned values — e.g. lift L +3.0 / R +0.4, trim roll −2.4 — and the next run started with them;
(2) the previous run's cycle-mean roll (+8° after a fall) was used as the centre of the first
excursions. Fixes: a tilt stop reverts trims and lift deltas to the last cycle with a settled rhythm
(`revertToLastGood`), `beginRun` clears the cycle means, and the pitch target default is now 1.0°
(operator found it removes the forward start-up falls).
