#pragma once

#include <cstdint>

namespace tilt::config::motion {

// Values used by the currently verified incremental servo test.
inline constexpr std::int32_t kFineStepTicks = 1;
inline constexpr std::int32_t kCoarseStepTicks = 20;
inline constexpr std::uint16_t kFineServoSpeedRaw = 0;
inline constexpr std::uint16_t kCoarseServoSpeedRaw = 0;

// Provisional values for the upcoming pose-play path.
inline constexpr std::uint8_t kDefaultServoAccelerationRaw = 10;
inline constexpr std::uint32_t kPoseDurationMs = 3000;
inline constexpr std::uint32_t kSingleJointDurationMs = 1000;
inline constexpr std::uint16_t kPoseUpdatePeriodMs = 100;
inline constexpr float kMaximumJointMonkeyDeltaDegrees = 5.0f;

}  // namespace tilt::config::motion
