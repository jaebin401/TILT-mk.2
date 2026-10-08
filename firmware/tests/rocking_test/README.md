# TILT mk.2 rocking test

This ESP-IDF test app runs the same `tilt_gait::RockingGait` C++ state machine
that the MuJoCo bridge loads. It is a rocking/brief-SSP test, not yet a forward
walking controller.

## Presets

| Preset | Stance hip roll | Swing hip roll | Swing hip/knee | Half-cycle | Full cycle |
|---|---:|---:|---:|---:|---:|
| `CONSERVATIVE` | left `-1.5°`, right `+1.5°` | left `-8°`, right `+8°` | `-14° / +14°` | 500 ms | 1.00 s |
| `VISUAL` | left `-2°`, right `+2°` | left `-10°`, right `+10°` | `-16° / +16°` | 460 ms | 0.92 s |

The stance and swing hip-roll targets are intentionally asymmetric. The
loaded hip shifts the torso over the support foot while the unloaded hip tucks
the swing foot toward the centerline. After touchdown the gait transfers
directly to the opposite support side; there is no recenter-and-wait phase.

Neither preset is certified safe on hardware. The current MuJoCo motor model
still approaches the STS3215-C001 stall envelope and exceeds rated torque.
Verify joint directions, zero ticks, limits, supply voltage and current before
allowing either foot to support the robot without an overhead restraint.

## Build and flash

Run these commands from an ESP-IDF-enabled shell:

```bash
cd firmware/tests/rocking_test
idf.py set-target esp32s3
idf.py build
idf.py -p PORT flash monitor
```

This environment did not have `IDF_PATH`/`idf.py`, so only the platform-neutral
gait core and MuJoCo bridge were compiled here. The ESP-IDF app still needs one
build on the development machine before flashing.

## Console controls

```text
o       arm and hold the measured pose
s       move to stand over 2 seconds
r       start continuous rocking
x       stop and return to stand
1       select CONSERVATIVE preset
2       select VISUAL preset (suspended robot only)
-/+     shorten/lengthen half-cycle by 20 ms (320..800 ms)
m       cycle PRESET / PITCH_ONLY / CUSTOM hip-roll mode
e       edit custom hip-roll pose and pitch lift height
p       print status
SPACE   emergency torque-off
```

Period changes are accepted only while the robot is not moving. `-` makes the
gait faster and `+` makes it slower. Selecting preset `1` or `2` restores that
preset's default period. Values at or below 360 ms are an aggressive suspended
test range, not a verified load-bearing command.

Recommended first run: suspend the torso, keep both feet clear of obstacles,
press `o`, then `s`, select `1`, and finally press `r`. Keep the terminal focused
so SPACE is immediately available.

The MPU6050 is read when present. A measured roll or pitch magnitude over 35°
causes torque-off. That threshold is only an emergency backstop; IMU axis
alignment must be verified before relying on it.

## MuJoCo visualization using the same C++ gait

From the repository root:

```bash
cmake -S simulation -B simulation/build
cmake --build simulation/build
./.venv/bin/python ./.venv/bin/mjpython simulation/rocking_sim.py \
  --gait-source cpp --preset visual --forever
```

Close the viewer window or press Ctrl+C to stop.
