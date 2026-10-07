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

View: `mjpython -m mujoco.viewer --mjcf=model/mjcf/scene_view.xml`
