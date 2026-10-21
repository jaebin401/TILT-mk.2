#pragma once

// TILT mk.2 leg kinematics (hip roll -> hip pitch -> knee pitch -> ankle pitch).
//
// The chain constants come from include/tilt/kinematics/RobotModel.h, which is
// generated from model/tilt.urdf by model/tools/generate_robot_model.py.
// Do not hand-edit link lengths here; regenerate the model instead.
//
// Ankle (2026-10-13): the URDF does not have the ankle joint yet. Until it is
// regenerated, the ankle is modelled on top of the URDF leg chain:
//   * axis parallel to the knee axis, on the calf CALF_LENGTH_MM from the knee
//   * the sole reference is directly below that axis by FOOT_LENGTH_MM when the
//     sole is flat, and
//   * the foot rotates about it by ankle_pitch + (ANKLE_FIXED_RAD -
//     model::SOLE_PITCH_AT_ZERO_RAD), so that ankle_pitch = 0 gives a sole that
//     is flat when the leg is straight (tilt_config.h).
//   Link masses are the URDF ones (the ankle servo is not in the CoM yet).
//
// Frames and units
//   * Base frame B: torso axes (+x forward, +y left, +z up). Origin at the
//     midpoint of the two hip points H (where each leg's hip-roll and
//     hip-pitch axes intersect). Lengths in mm, angles in rad.
//   * theta[NUM_LEG_JOINTS] = {hip_roll, hip_pitch, knee_pitch, ankle_pitch}
//     for one leg, using the firmware logical signs (tilt_config.h):
//       hip_roll  +  foot moves to the robot's left (+y)
//       hip_pitch +  leg swings backward
//       knee_pitch + knee flexes
//       ankle_pitch + toes down
//   * The "sole" point is the foot-pad centre on its contact plane. Its frame
//     has z up and x forward when the foot
//     is flat (hip_pitch + knee_pitch + ankle_pitch = -ANKLE_FIXED_RAD = 0).

#include "tilt_config.h"
#include "tilt/kinematics/RobotModel.h"

namespace tilt {

enum class Leg { LEFT = 0, RIGHT = 1 };

struct Vec3 {
    float x;
    float y;
    float z;
};

struct Mat3 {
    float m[3][3];  // Row-major.
};

struct Pose {
    Mat3 R;  // Orientation in B.
    Vec3 p;  // Origin in B (mm).
};

// Link frames of one leg in B. Each frame is the URDF child-link frame after
// its joint rotation (hip = hip-roll output, thigh = hip-pitch output,
// calf = knee output, ankle = foot frame on the ankle axis after the ankle
// rotation, sole = sole frame).
struct LegFrames {
    Pose hip;
    Pose thigh;
    Pose calf;
    Pose ankle;
    Pose sole;
};

struct IkResult {
    float theta[NUM_LEG_JOINTS];
    bool reachable;  // False when the target was clamped to the workspace.
};

// Full forward kinematics of one leg.
LegFrames fk_leg(Leg leg, const float theta[NUM_LEG_JOINTS]);

// Sole point in B.
Vec3 fk_foot(Leg leg, const float theta[NUM_LEG_JOINTS]);

// Sole pitch about +y relative to the torso (0 = sole parallel to the torso's
// xy plane; positive = toes down). Equals the sole frame's pitch in fk_leg.
float sole_pitch_rad(const float theta[NUM_LEG_JOINTS]);

// Closed-form inverse kinematics for the sole point with a given sole pitch
// (default 0 = sole parallel to the torso). Uses the knee-flexed branch
// (knee_pitch > ~2 deg). Out-of-reach targets are projected onto the
// workspace boundary, the angles stay finite and reachable is false.
IkResult ik_foot(Leg leg, const Vec3& sole_target, float sole_pitch = 0.0f);

// Sagittal IK with hip roll and hip pitch held at caller-selected values.
// Knee and ankle pitch are solved so the sole contact point reaches x/z;
// sole orientation is intentionally left free. The hints select the closest
// continuous two-link branch. `reachable` is false when the requested point
// is outside the fixed-hip workspace or outside the fixed-roll plane.
IkResult ik_foot_fixed_hip_pitch(Leg leg, const Vec3& sole_target,
                                 float hip_roll, float hip_pitch,
                                 float knee_hint, float ankle_hint);

// Transforms a point given in a pose's local frame into B.
Vec3 transform_point(const Pose& pose, const Vec3& local);

// Identity pose of the base frame B.
Pose base_pose();

// Applies one generated joint: child = parent * T(origin) * rotation * Rot(axis, q).
Pose joint_child_pose(const Pose& parent, const model::JointSpec& joint, float q);

// Clamps in place to the logical joint limits for the selected leg.
bool clamp_to_limits(Leg leg, float theta[NUM_LEG_JOINTS]);

}  // namespace tilt
