#pragma once

// TILT mk.2 leg kinematics (hip roll -> hip pitch -> knee pitch, fixed ankle).
//
// The chain constants come from include/tilt/kinematics/RobotModel.h, which is
// generated from model/tilt.urdf by model/tools/generate_robot_model.py.
// Do not hand-edit link lengths here; regenerate the model instead.
//
// Frames and units
//   * Base frame B: torso axes (+x forward, +y left, +z up). Origin at the
//     midpoint of the two hip points H (where each leg's hip-roll and
//     hip-pitch axes intersect). Lengths in mm, angles in rad.
//   * theta[3] = {hip_roll, hip_pitch, knee_pitch} for one leg, using the
//     firmware logical signs (tilt_config.h):
//       hip_roll  +  foot moves to the robot's left (+y)
//       hip_pitch +  leg swings backward
//       knee_pitch + knee flexes
//   * The "sole" point is the URDF sole frame origin (under the calf, on the
//     foot's bottom surface). Its frame has z up and x forward when the foot
//     is flat (stand pose: hip_pitch + knee_pitch = +20 deg).

#include "tilt_config.h"

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
// calf = knee output, sole = fixed sole frame).
struct LegFrames {
    Pose hip;
    Pose thigh;
    Pose calf;
    Pose sole;
};

struct IkResult {
    float theta[3];
    bool reachable;  // False when the target was clamped to the workspace.
};

// Full forward kinematics of one leg.
LegFrames fk_leg(Leg leg, const float theta[3]);

// Sole point in B.
Vec3 fk_foot(Leg leg, const float theta[3]);

// Sole pitch about +y relative to the torso (0 = sole parallel to the torso's
// xy plane; positive = toes down). Equals the sole frame's pitch in fk_leg.
float sole_pitch_rad(const float theta[3]);

// Closed-form inverse kinematics for the sole point. Uses the knee-flexed
// branch (knee_pitch > ~2 deg). Out-of-reach targets are projected onto the
// workspace boundary, the angles stay finite and reachable is false.
IkResult ik_foot(Leg leg, const Vec3& sole_target);

// Transforms a point given in a pose's local frame into B.
Vec3 transform_point(const Pose& pose, const Vec3& local);

// Clamps in place to the logical joint limits for the selected leg.
bool clamp_to_limits(Leg leg, float theta[3]);

}  // namespace tilt
