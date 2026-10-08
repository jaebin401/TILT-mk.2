"""Generate the firmware kinematic/mass model from model/tilt.urdf.

Outputs (paths relative to the repository root):
  firmware/components/tilt_kinematics/include/tilt/kinematics/RobotModel.h
  firmware/tests/host_test/kinematics_reference.h   (only with --reference; needs mujoco)

Usage (from the repository root):
  python3 model/tools/generate_robot_model.py [--reference]

Frames and units in the generated header
  * Base frame B: torso axes (+x forward, +y left, +z up), origin at the midpoint of
    the two hip points H. H is where a leg's hip-roll and hip-pitch axes intersect.
  * Lengths in millimetres, masses in kilograms, angles in radians.
  * Joint angle signs are the URDF signs, which equal the firmware logical signs
    (see model/tools/fix_tilt_model.py and tilt_config.h).
  * Leg index 0 = LEFT, 1 = RIGHT (same as tilt::Leg and JOINT_LIMIT).
"""
import argparse
import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
URDF = os.path.join(REPO, "model", "tilt.urdf")
MJCF = os.path.join(REPO, "model", "mjcf", "scene.xml")
OUT_HEADER = os.path.join(
    REPO, "firmware", "components", "tilt_kinematics", "include", "tilt",
    "kinematics", "RobotModel.h")
OUT_REFERENCE = os.path.join(
    REPO, "firmware", "tests", "host_test", "kinematics_reference.h")

LEGS = ("l", "r")  # index 0 = LEFT, 1 = RIGHT
LEG_JOINTS = ("hip_roll", "hip_pitch", "knee_pitch")
LEG_LINKS = ("hip", "thigh", "calf")


# ---------------------------------------------------------------- URDF model
def rpy_to_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def axis_angle(axis, angle):
    a = np.asarray(axis, dtype=float)
    a = a / np.linalg.norm(a)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * k @ k


class Urdf:
    def __init__(self, path):
        root = ET.parse(path).getroot()
        self.joints = {}
        self.links = {}
        for j in root.findall("joint"):
            origin = j.find("origin")
            axis = j.find("axis")
            self.joints[j.get("name")] = {
                "type": j.get("type"),
                "parent": j.find("parent").get("link"),
                "child": j.find("child").get("link"),
                "xyz": np.array([float(v) for v in origin.get("xyz").split()]),
                "R": rpy_to_matrix(*[float(v) for v in origin.get("rpy").split()]),
                "axis": np.array([float(v) for v in axis.get("xyz").split()])
                if axis is not None else np.array([0.0, 0.0, 1.0]),
            }
        for link in root.findall("link"):
            inertial = link.find("inertial")
            if inertial is None:
                self.links[link.get("name")] = (0.0, np.zeros(3))
                continue
            self.links[link.get("name")] = (
                float(inertial.find("mass").get("value")),
                np.array([float(v) for v in inertial.find("origin").get("xyz").split()]),
            )
        self.joint_of_child = {j["child"]: n for n, j in self.joints.items()}

    def pose(self, link, q):
        """Pose of `link` in the torso frame (R, p in metres)."""
        if link == "torso":
            return np.eye(3), np.zeros(3)
        name = self.joint_of_child[link]
        j = self.joints[name]
        parent_R, parent_p = self.pose(j["parent"], q)
        R = parent_R @ j["R"]
        p = parent_p + parent_R @ j["xyz"]
        if j["type"] != "fixed":
            R = R @ axis_angle(j["axis"], q.get(name, 0.0))
        return R, p


# ------------------------------------------------------------ derived values
def derive(urdf):
    q0 = {}
    hips = []
    for side in LEGS:
        R_roll, p_roll = urdf.pose(f"{side}_hip", q0)
        R_pitch, p_pitch = urdf.pose(f"{side}_thigh", q0)
        roll_axis = R_roll @ urdf.joints[f"{side}_hip_roll"]["axis"]
        pitch_axis = R_pitch @ urdf.joints[f"{side}_hip_pitch"]["axis"]
        # Closest point between the two axis lines (they intersect on mk.2).
        w0 = p_roll - p_pitch
        a, b, c = roll_axis @ roll_axis, roll_axis @ pitch_axis, pitch_axis @ pitch_axis
        d, e = roll_axis @ w0, pitch_axis @ w0
        den = a * c - b * b
        s = (b * e - c * d) / den
        t = (a * e - b * d) / den
        h1 = p_roll + s * roll_axis
        h2 = p_pitch + t * pitch_axis
        hips.append(0.5 * (h1 + h2))
        miss = np.linalg.norm(h1 - h2)
        if miss > 1e-5:
            sys.exit(f"{side}: hip roll and pitch axes miss by {miss * 1000:.3f} mm")
        if np.linalg.norm(roll_axis - [1, 0, 0]) > 1e-4 or np.linalg.norm(pitch_axis - [0, 1, 0]) > 1e-4:
            sys.exit(f"{side}: unexpected axis directions {roll_axis} {pitch_axis}")
    base = 0.5 * (hips[0] + hips[1])
    return base, hips


def leg_planar_parameters(urdf, side, hip):
    """Planar 2-link parameters in the roll frame (zero pose)."""
    q0 = {}
    _, p_knee = urdf.pose(f"{side}_calf", q0)
    _, p_sole = urdf.pose(f"{side}_sole", q0)
    # At q = 0 the roll frame is aligned with the torso frame.
    thigh = p_knee - hip
    shank = p_sole - p_knee
    lateral = (p_sole - hip)[1]
    # Pitch/knee axes are +y, so y components never change: lateral is constant.
    return (np.array([thigh[0], thigh[2]]), np.array([shank[0], shank[2]]),
            lateral)


# -------------------------------------------------------------- C++ emitters
def f(v):
    if abs(v) < 5e-12:
        return "0.0f"
    text = f"{v:.9g}"
    if not any(ch in text for ch in ".en"):
        text += ".0"
    return text + "f"


def vec(v):
    return "{" + ", ".join(f(x) for x in v) + "}"


def mat(m):
    return "{" + ", ".join(vec(row) for row in m) + "}"


def joint_spec(urdf, name, base_shift=None):
    j = urdf.joints[name]
    origin = j["xyz"] * 1000.0
    if base_shift is not None:
        origin = origin - base_shift * 1000.0
    return "{" + f"{vec(origin)}, {mat(j['R'])}, {vec(j['axis'])}" + "}"


def inertial(urdf, link, shift=None):
    mass, com = urdf.links[link]
    com = com * 1000.0
    if shift is not None:
        com = com - shift * 1000.0
    return "{" + f"{f(mass)}, {vec(com)}" + "}"


def emit_header(urdf):
    base, hips = derive(urdf)
    planar = [leg_planar_parameters(urdf, s, h) for s, h in zip(LEGS, hips)]
    total_mass = sum(m for m, _ in urdf.links.values())

    leg_joints = ",\n    ".join(
        "{" + ", ".join(joint_spec(urdf, f"{s}_{j}", base if j == "hip_roll" else None)
                        for j in LEG_JOINTS) + "}"
        for s in LEGS)
    sole = ",\n    ".join(joint_spec(urdf, f"{s}_sole_fixed") for s in LEGS)
    leg_links = ",\n    ".join(
        "{" + ", ".join(inertial(urdf, f"{s}_{l}") for l in LEG_LINKS) + "}"
        for s in LEGS)
    arm_joints = ",\n    ".join(
        "{" + joint_spec(urdf, f"{s}_arm_pitch", base) + ", "
        + joint_spec(urdf, f"{s}_arm_roll") + "}" for s in LEGS)
    arm_links = ",\n    ".join(
        "{" + inertial(urdf, f"{s}_shoulder") + ", " + inertial(urdf, f"{s}_arm") + "}"
        for s in LEGS)

    hip_center = ",\n    ".join(vec((h - base) * 1000.0) for h in hips)
    thigh_vec = ", ".join(vec(p[0] * 1000.0) for p in planar)
    shank_vec = ", ".join(vec(p[1] * 1000.0) for p in planar)
    lateral = vec([p[2] * 1000.0 for p in planar])

    # Sole pitch at q = 0 (rotation about +y of the sole frame w.r.t. torso).
    R_sole0, _ = urdf.pose("l_sole", {})
    sole_pitch0 = math.atan2(R_sole0[0, 2], R_sole0[2, 2])

    return f"""#pragma once

// AUTO-GENERATED by model/tools/generate_robot_model.py from model/tilt.urdf.
// Do not edit by hand: change the URDF and regenerate.
//
// Base frame B: torso axes (+x forward, +y left, +z up), origin at the midpoint
// of the two hip points H (intersection of each leg's hip-roll and hip-pitch
// axes). Lengths in mm, mass in kg, angles in rad. Leg index 0 = LEFT, 1 = RIGHT.
// A joint transform is: child = parent * Translate(origin) * rotation * Rot(axis, q).

namespace tilt::model {{

struct JointSpec {{
    float origin_mm[3];
    float rotation[3][3];
    float axis[3];
}};

struct LinkInertial {{
    float mass_kg;
    float com_mm[3];  // In the link frame.
}};

// Origin of B expressed in the URDF torso frame.
inline constexpr float BASE_IN_TORSO_MM[3] = {vec(base * 1000.0)};

// [leg][hip_roll, hip_pitch, knee_pitch]; hip_roll's parent is B.
inline constexpr JointSpec LEG_JOINT[2][3] = {{
    {leg_joints},
}};

// Fixed calf -> sole frame (sole frame: z up and x forward when the foot is flat).
inline constexpr JointSpec SOLE_FIXED[2] = {{
    {sole},
}};

// [leg][hip, thigh, calf] inertials in their link frames.
inline constexpr LinkInertial LEG_LINK[2][3] = {{
    {leg_links},
}};

// Torso inertial in B.
inline constexpr LinkInertial TORSO = {inertial(urdf, "torso", base)};

// Head (parent B) and arms [leg-side][pitch (parent B), roll (parent shoulder)].
inline constexpr JointSpec HEAD_PITCH = {joint_spec(urdf, "head_pitch", base)};
inline constexpr LinkInertial HEAD = {inertial(urdf, "head")};
inline constexpr JointSpec ARM_JOINT[2][2] = {{
    {arm_joints},
}};
inline constexpr LinkInertial ARM_LINK[2][2] = {{
    {arm_links},
}};

inline constexpr float TOTAL_MASS_KG = {f(total_mass)};

// ---- Analytic IK parameters (derived from the same chain) ----
// Hip point H of each leg in B.
inline constexpr float HIP_POINT_MM[2][3] = {{
    {hip_center},
}};
// Sole lateral offset from H in the roll frame (+ = left). Constant because the
// pitch and knee axes are parallel to the roll frame's y axis.
inline constexpr float SOLE_LATERAL_MM[2] = {lateral};
// Sagittal (x, z) vectors at q = 0: H -> knee, and knee -> sole.
inline constexpr float THIGH_XZ_MM[2][2] = {{{thigh_vec}}};
inline constexpr float SHANK_XZ_MM[2][2] = {{{shank_vec}}};
// Sole pitch (about +y) when hip_pitch + knee_pitch = 0.
inline constexpr float SOLE_PITCH_AT_ZERO_RAD = {f(sole_pitch0)};

}}  // namespace tilt::model
"""


# ---------------------------------------------------------- reference fixture
def emit_reference(urdf, samples=240, seed=7):
    import mujoco  # noqa: deferred import, only needed for --reference

    base, _ = derive(urdf)
    model = mujoco.MjModel.from_xml_path(MJCF)
    data = mujoco.MjData(model)
    qadr = {model.joint(i).name: model.jnt_qposadr[i] for i in range(model.njnt)}
    site = {s: model.site(f"{s}_sole").id for s in LEGS}
    rng = np.random.default_rng(seed)
    lo = np.deg2rad([-13.0, -70.0, 0.0])
    hi = np.deg2rad([17.0, 20.0, 75.0])

    rows = []
    max_urdf_err = 0.0
    for k in range(samples):
        theta = rng.uniform(lo, hi, size=(2, 3))
        if k == 0:
            theta[:] = np.deg2rad([0.0, -20.0, 40.0])
        mujoco.mj_resetData(model, data)
        data.qpos[3:7] = (1, 0, 0, 0)
        q = {}
        for leg, side in enumerate(LEGS):
            for i, jn in enumerate(LEG_JOINTS):
                data.qpos[qadr[f"{side}_{jn}"]] = theta[leg, i]
                q[f"{side}_{jn}"] = theta[leg, i]
        mujoco.mj_kinematics(model, data)
        mujoco.mj_comPos(model, data)
        sole_pos = []
        sole_rot = []
        for side in LEGS:
            p = (data.site_xpos[site[side]] - data.xpos[1] - base) * 1000.0
            R = data.site_xmat[site[side]].reshape(3, 3).copy()
            R_u, p_u = urdf.pose(f"{side}_sole", q)
            max_urdf_err = max(max_urdf_err, np.linalg.norm((p_u - base) * 1000.0 - p),
                               np.abs(R_u - R).max() * 100.0)
            sole_pos.append(p)
            sole_rot.append(R)
        com = data.subtree_com[1].copy()
        com_b = (com - data.xpos[1] - base) * 1000.0
        rows.append((theta, sole_pos, sole_rot, com_b))
    if max_urdf_err > 0.01:
        sys.exit(f"URDF and MJCF disagree: {max_urdf_err:.4f} mm")

    body = []
    for theta, sp, sr, com in rows:
        body.append("    {" + vec(theta[0]) + ", " + vec(theta[1]) + ", "
                    + vec(sp[0]) + ", " + vec(sp[1]) + ", "
                    + mat(sr[0]) + ", " + mat(sr[1]) + ", " + vec(com) + "},")
    return f"""#pragma once

// AUTO-GENERATED by model/tools/generate_robot_model.py --reference.
// Ground truth from MuJoCo (model/mjcf/scene.xml) for {samples} joint samples.
// Positions in mm in base frame B; arms/head at zero. Sample 0 is the stand pose.

namespace kinematics_reference {{

struct Sample {{
    float left_theta[3];
    float right_theta[3];
    float left_sole_mm[3];
    float right_sole_mm[3];
    float left_sole_R[3][3];
    float right_sole_R[3][3];
    float com_mm[3];
}};

inline constexpr Sample SAMPLES[] = {{
{chr(10).join(body)}
}};

}}  // namespace kinematics_reference
"""


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--reference", action="store_true",
                        help="also write the MuJoCo reference fixture for host tests")
    args = parser.parse_args()
    urdf = Urdf(URDF)
    os.makedirs(os.path.dirname(OUT_HEADER), exist_ok=True)
    with open(OUT_HEADER, "w") as fh:
        fh.write(emit_header(urdf))
    print(f"wrote {os.path.relpath(OUT_HEADER, REPO)}")
    if args.reference:
        with open(OUT_REFERENCE, "w") as fh:
            fh.write(emit_reference(urdf))
        print(f"wrote {os.path.relpath(OUT_REFERENCE, REPO)}")


if __name__ == "__main__":
    main()
