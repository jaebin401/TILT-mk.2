#pragma once

#include <array>
#include <cmath>
#include <cstdint>

#include "tilt_config.h"

namespace tilt::pose {

using JointArray = std::array<float, NUM_JOINTS>;

inline bool radToTick(int joint, float rad, std::uint16_t& tick_out) {
    if (joint < 0 || joint >= NUM_JOINTS || !std::isfinite(rad)) {
        return false;
    }

    const float raw = static_cast<float>(ZERO_TICK[joint]) +
                      static_cast<float>(JOINT_SIGN[joint]) *
                          rad / RAD_PER_TICK;
    const long rounded = std::lround(raw);
    if (rounded < SERVO_POS_MIN || rounded > SERVO_POS_MAX) {
        return false;
    }

    tick_out = static_cast<std::uint16_t>(rounded);
    return true;
}

inline float tickToRad(int joint, std::uint16_t tick) {
    if (joint < 0 || joint >= NUM_JOINTS) {
        return 0.0f;
    }
    return static_cast<float>(JOINT_SIGN[joint]) *
           (static_cast<int>(tick) - static_cast<int>(ZERO_TICK[joint])) *
           RAD_PER_TICK;
}

inline bool withinLimits(const JointArray& q, int& bad_joint) {
    for (int joint = 0; joint < NUM_JOINTS; ++joint) {
        const int leg = joint / 3;
        const int leg_joint = joint % 3;
        const JointLimit& limit = JOINT_LIMIT[leg][leg_joint];
        if (!std::isfinite(q[joint]) || q[joint] < limit.minimum_rad ||
            q[joint] > limit.maximum_rad) {
            bad_joint = joint;
            return false;
        }
    }
    bad_joint = -1;
    return true;
}

inline float footPitchRad(const JointArray& q, int leg) {
    if (leg < 0 || leg > 1) {
        return 0.0f;
    }
    const int base = leg * 3;
    return q[base + 1] + q[base + 2] + KNEE_OFFSET_RAD + ANKLE_FIXED_RAD;
}

}  // namespace tilt::pose
