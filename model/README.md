# TILT mk.2 robot model

- `tilt.urdf` — URDF (11 revolute joints, base `torso`, frames `l_sole` / `r_sole`)
- `mjcf/scene.xml` — MuJoCo, free-floating on a floor
- `mjcf/scene_view.xml` — MuJoCo, torso welded in the air (contacts off) for checking joints
- `meshes/` — merged per-link STL (visual == collision for now)
- `onshape/config.json` — onshape-to-robot export config (Onshape version URL, no secrets)
- `tools/fix_tilt_model.py` — post-processing applied after export (run in the export workspace)

Conventions (same as firmware `tilt_config.h`): +x forward, +y left, +z up; right-hand rule common to both sides.
hip_roll + = foot to robot's left, hip_pitch + = leg swings back, knee_pitch + = knee flexes.
Joint 0 = legs straight (CAD pose). Firmware stand pose (hip −20°, knee +40°) gives flat soles,
hip-pitch axis → sole = 136.61 mm.

Limits: legs from `tilt_config.h` JOINT_LIMIT (roll provisional); arm ±90° and head ±45° are placeholders.
effort / velocity are approximate datasheet values (STS3215, SG90, MG996). Total CAD mass 1239 g (battery etc. not included).

The calf collision meshes are disabled for contact and replaced by explicit 70 x 38 x 5 mm rectangular
sole proxies. This keeps the rendered CAD geometry while making contact repeatable enough for rocking tests.

## Dynamic rocking / brief SSP test

The runner in `simulation/rocking_sim.py` replaces the six leg position actuators at runtime with a
torque controller constrained by the STS3215-C001 datasheet at 7.4 V:

- no-load speed: 52 RPM (0.192 s/60°)
- stall torque: 19.5 kgf·cm = 1.912 N·m
- rated torque: 5 kgf·cm = 0.490 N·m
- stall current: 2.5 A; vendor over-current threshold: 2 A for 2 s

The simulator uses the stall value only as a short-duration torque-speed envelope. It separately reports
time above rated torque and the 2 A protection threshold; stall torque is not treated as continuously safe.

```bash
./.venv/bin/pip install mujoco numpy matplotlib
./.venv/bin/python simulation/rocking_sim.py --search
./.venv/bin/python simulation/rocking_sim.py --support both --cycles 3 \
  --roll-deg 11 --roll-time 0.15 --lift-deg 10 --lift-time 0.20 --lift-offset -0.05 \
  --plot simulation/results/rocking_demo.png
```

On macOS, launch the interactive viewer through MuJoCo's `mjpython` wrapper:

```bash
./.venv/bin/python ./.venv/bin/mjpython simulation/rocking_sim.py --support both --viewer
```

Repeat continuously until the viewer window is closed:

```bash
./.venv/bin/python ./.venv/bin/mjpython simulation/rocking_sim.py --support both --forever
```

This is a best-case 7.4 V source model. Measure loaded bus voltage, joint zero offsets, and at least one
loaded 60° motion before using the simulation result as a hardware command table.

The first coarse sweep found brief SSP candidates, but every mechanically successful candidate touched
the 19.5 kgf·cm stall envelope. Treat those results as proof that the motion is dynamically possible, not
as a hardware-safe command table; the hip-roll current/torque model must be calibrated on the real robot.

View: `./.venv/bin/python ./.venv/bin/mjpython -m mujoco.viewer --mjcf=model/mjcf/scene_view.xml`
