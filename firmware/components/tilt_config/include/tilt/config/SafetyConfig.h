#pragma once

#include <array>
#include <cstddef>

#include "tilt/config/RobotConfig.h"

namespace tilt::config::safety {
namespace detail {

constexpr float degreesToRadians(float degrees) {
    return degrees * 0.01745329251994329577f;
}

}  // namespace detail

struct JointPositionLimit {
    float minimum_radians;
    float maximum_radians;
};

// Same measured logical limits as the gait and pose paths.
inline constexpr std::array<JointPositionLimit, robot::kJointCount>
    kJointPositionLimits = {{
        {JOINT_LIMIT[0][0].minimum_rad, JOINT_LIMIT[0][0].maximum_rad},
        {JOINT_LIMIT[0][1].minimum_rad, JOINT_LIMIT[0][1].maximum_rad},
        {JOINT_LIMIT[0][2].minimum_rad, JOINT_LIMIT[0][2].maximum_rad},
        {JOINT_LIMIT[0][3].minimum_rad, JOINT_LIMIT[0][3].maximum_rad},
        {JOINT_LIMIT[1][0].minimum_rad, JOINT_LIMIT[1][0].maximum_rad},
        {JOINT_LIMIT[1][1].minimum_rad, JOINT_LIMIT[1][1].maximum_rad},
        {JOINT_LIMIT[1][2].minimum_rad, JOINT_LIMIT[1][2].maximum_rad},
        {JOINT_LIMIT[1][3].minimum_rad, JOINT_LIMIT[1][3].maximum_rad},
    }};

inline constexpr std::array<float, robot::kJointCount>
    kMaximumJointVelocityRadiansPerSecond = {
        detail::degreesToRadians(35.0f),
        detail::degreesToRadians(35.0f),
        detail::degreesToRadians(35.0f),
        detail::degreesToRadians(35.0f),
        detail::degreesToRadians(35.0f),
        detail::degreesToRadians(35.0f),
        detail::degreesToRadians(35.0f),
        detail::degreesToRadians(35.0f),
    };

inline constexpr bool kJointSafetyLimitsVerified = false;

}  // namespace tilt::config::safety
