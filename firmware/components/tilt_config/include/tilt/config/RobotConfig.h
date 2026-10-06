#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace tilt::config::robot {

inline constexpr std::size_t kJointCount = 6;

enum class JointIndex : std::size_t {
    kLeftHipYaw = 0,
    kLeftHipPitch,
    kLeftKneePitch,
    kRightHipYaw,
    kRightHipPitch,
    kRightKneePitch,
};

// All joint-indexed tables in tilt_config use this order.
inline constexpr std::array<std::string_view, kJointCount> kJointNames = {
    "LHY", "LHP", "LKP", "RHY", "RHP", "RKP",
};

inline constexpr std::array<std::uint8_t, kJointCount> kServoIds = {
    11, 12, 13, 21, 22, 23,
};

inline constexpr std::uint16_t kPositionReadFallbackTicks = 2048;

struct ServoCalibration {
    std::int8_t direction;
    std::uint16_t zero_ticks;
    std::uint16_t minimum_ticks;
    std::uint16_t maximum_ticks;
};

// Provisional values from the pose-test draft. Do not use these as confirmed
// mechanical limits until every installed servo has been calibrated.
inline constexpr std::array<ServoCalibration, kJointCount>
    kServoCalibrations = {{
        {+1, 2047, 1500, 2600},  // LHY / ID 11
        {+1, 2047, 1500, 2600},  // LHP / ID 12
        {+1, 2047, 1500, 2600},  // LKP / ID 13
        {+1, 2047, 1500, 2600},  // RHY / ID 21
        {+1, 2047, 1500, 2600},  // RHP / ID 22
        {+1, 2047, 1500, 2600},  // RKP / ID 23
    }};

inline constexpr bool kServoCalibrationVerified = false;

}  // namespace tilt::config::robot
