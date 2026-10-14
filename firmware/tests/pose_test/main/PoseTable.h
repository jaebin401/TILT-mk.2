#pragma once

#include <cstdint>

#include "JointMap.h"

namespace tilt::pose {

enum class Side { LEFT, RIGHT };

inline constexpr float kShiftRollDeg = 8.0f;
inline constexpr float kLiftHipDeltaDeg = -15.0f;
inline constexpr float kLiftKneeDeltaDeg = +15.0f;

inline constexpr std::uint32_t kHomeDurationMs = 2000;
inline constexpr std::uint32_t kStandDurationMs = 2000;
inline constexpr std::uint32_t kShiftDurationMs = 600;
inline constexpr std::uint32_t kLiftDurationMs = 400;

inline constexpr float degreesToRadians(float degrees) {
    return degrees * DEG2RAD;
}

inline JointArray homePose() {
    return {};
}

inline JointArray standPose() {
    JointArray pose{};
    for (int joint = 0; joint < NUM_JOINTS; ++joint) {
        pose[joint] = ZERO_POSE_RAD[joint];
    }
    return pose;
}

inline JointArray shiftPose(Side support) {
    JointArray pose = standPose();
    const float roll = degreesToRadians(
        support == Side::RIGHT ? +kShiftRollDeg : -kShiftRollDeg);
    pose[L_HIP_ROLL] = roll;
    pose[R_HIP_ROLL] = roll;
    return pose;
}

inline JointArray liftPose(Side support) {
    JointArray pose = shiftPose(support);
    const int swing_leg = support == Side::RIGHT ? 0 : 1;
    const int hip_pitch = jointIndex(swing_leg, 1);
    const int knee_pitch = jointIndex(swing_leg, 2);
    const int ankle_pitch = jointIndex(swing_leg, 3);
    const float hip_delta = degreesToRadians(kLiftHipDeltaDeg);
    const float knee_delta = degreesToRadians(kLiftKneeDeltaDeg);
    pose[hip_pitch] += hip_delta;
    pose[knee_pitch] += knee_delta;
    pose[ankle_pitch] -= hip_delta + knee_delta;
    return pose;
}

}  // namespace tilt::pose
