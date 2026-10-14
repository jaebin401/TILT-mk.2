#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>

#include "tilt_config.h"

namespace tilt::config::robot {

inline constexpr std::size_t kJointCount = NUM_JOINTS;

enum class JointIndex : std::size_t {
    kLeftHipRoll = 0,
    kLeftHipPitch,
    kLeftKneePitch,
    kLeftAnklePitch,
    kRightHipRoll,
    kRightHipPitch,
    kRightKneePitch,
    kRightAnklePitch,
};

// All joint-indexed tables in tilt_config use this order.
inline constexpr std::array<std::string_view, kJointCount> kJointNames = {
    "LHR", "LHP", "LKP", "LAP", "RHR", "RHP", "RKP", "RAP",
};

inline constexpr std::array<std::uint8_t, kJointCount> kServoIds = {
    SERVO_ID[0], SERVO_ID[1], SERVO_ID[2], SERVO_ID[3],
    SERVO_ID[4], SERVO_ID[5], SERVO_ID[6], SERVO_ID[7],
};

struct ServoCalibration {
    std::int8_t direction;
    std::uint16_t zero_ticks;
    std::uint16_t minimum_ticks;
    std::uint16_t maximum_ticks;
};

namespace detail {

constexpr std::uint16_t jointRadiansToTicks(std::size_t joint,
                                            float radians) {
    const float raw = static_cast<float>(ZERO_TICK[joint]) +
                      static_cast<float>(JOINT_SIGN[joint]) *
                          radians / RAD_PER_TICK;
    const int rounded = static_cast<int>(raw + 0.5f);
    return static_cast<std::uint16_t>(
        rounded < SERVO_POS_MIN ? SERVO_POS_MIN :
        rounded > SERVO_POS_MAX ? SERVO_POS_MAX : rounded);
}

constexpr ServoCalibration makeServoCalibration(std::size_t joint) {
    const JointLimit& limit = jointLimit(static_cast<int>(joint));
    const std::uint16_t first = jointRadiansToTicks(joint, limit.minimum_rad);
    const std::uint16_t second = jointRadiansToTicks(joint, limit.maximum_rad);
    return {JOINT_SIGN[joint], ZERO_TICK[joint],
            first < second ? first : second,
            first < second ? second : first};
}

}  // namespace detail

// Mirrors the shared calibration used by the gait and pose paths. Mechanical
// joint limits remain in tilt::JOINT_LIMIT; these raw bounds only protect the
// incremental servo console from leaving the STS3215 position range.
inline constexpr std::array<ServoCalibration, kJointCount>
    kServoCalibrations = {{
        detail::makeServoCalibration(0),
        detail::makeServoCalibration(1),
        detail::makeServoCalibration(2),
        detail::makeServoCalibration(3),
        detail::makeServoCalibration(4),
        detail::makeServoCalibration(5),
        detail::makeServoCalibration(6),
        detail::makeServoCalibration(7),
    }};

inline constexpr bool kServoCalibrationVerified = false;

}  // namespace tilt::config::robot
