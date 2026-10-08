#pragma once

// Whole-body center of mass from the URDF mass model (tilt/kinematics/RobotModel.h).
// Platform-neutral: no ESP-IDF dependencies.

#include "tilt_config.h"
#include "tilt_kinematics.h"

namespace tilt::estimation {

// Non-leg joints. Index 0 = LEFT, 1 = RIGHT. Defaults are the URDF zero pose.
struct UpperBodyPose {
    float head_pitch_rad = 0.0f;
    float arm_pitch_rad[2] = {0.0f, 0.0f};
    float arm_roll_rad[2] = {0.0f, 0.0f};
};

// leg_theta uses the firmware joint order (tilt::JointIndex):
// {L_HIP_ROLL, L_HIP_PITCH, L_KNEE_PITCH, R_HIP_ROLL, R_HIP_PITCH, R_KNEE_PITCH}.
// Returns the CoM in base frame B (mm).
Vec3 center_of_mass(const float leg_theta[NUM_JOINTS],
                    const UpperBodyPose& upper = UpperBodyPose{});

// Total model mass (kg).
float total_mass_kg();

}  // namespace tilt::estimation
