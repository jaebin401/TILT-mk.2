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

// Provisional conservative limits from the pose-test draft.
inline constexpr std::array<JointPositionLimit, robot::kJointCount>
    kJointPositionLimits = {{
        {detail::degreesToRadians(-10.0f), detail::degreesToRadians(+10.0f)},
        {detail::degreesToRadians(-35.0f), detail::degreesToRadians(+5.0f)},
        {detail::degreesToRadians(-5.0f), detail::degreesToRadians(+40.0f)},
        {detail::degreesToRadians(-10.0f), detail::degreesToRadians(+10.0f)},
        {detail::degreesToRadians(-35.0f), detail::degreesToRadians(+5.0f)},
        {detail::degreesToRadians(-5.0f), detail::degreesToRadians(+40.0f)},
    }};

inline constexpr std::array<float, robot::kJointCount>
    kMaximumJointVelocityRadiansPerSecond = {
        detail::degreesToRadians(35.0f),
        detail::degreesToRadians(35.0f),
        detail::degreesToRadians(35.0f),
        detail::degreesToRadians(35.0f),
        detail::degreesToRadians(35.0f),
        detail::degreesToRadians(35.0f),
    };

inline constexpr bool kJointSafetyLimitsVerified = false;

}  // namespace tilt::config::safety
