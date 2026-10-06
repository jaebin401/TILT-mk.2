#pragma once

#include <cstdint>

#include "JointMap.h"

namespace tilt::pose {

enum class Side { LEFT, RIGHT };

inline constexpr float kStandRollDeg = 0.0f;
inline constexpr float kStandHipDeg = -20.0f;
inline constexpr float kStandKneeDeg = +40.0f;

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
    const float roll = degreesToRadians(kStandRollDeg);
    const float hip = degreesToRadians(kStandHipDeg);
    const float knee = degreesToRadians(kStandKneeDeg);
    return {roll, hip, knee, roll, hip, knee};
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
    const int swing_base = support == Side::RIGHT ? L_HIP_ROLL
                                                   : R_HIP_ROLL;
    pose[swing_base + 1] += degreesToRadians(kLiftHipDeltaDeg);
    pose[swing_base + 2] += degreesToRadians(kLiftKneeDeltaDeg);
    return pose;
}

}  // namespace tilt::pose
