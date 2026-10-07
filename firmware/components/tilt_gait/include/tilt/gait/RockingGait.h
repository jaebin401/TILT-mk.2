#pragma once

#include <array>
#include <cstdint>

#include "tilt_config.h"

namespace tilt::gait {

using JointArray = std::array<float, NUM_JOINTS>;

enum class Side : std::uint8_t { kLeft = 0, kRight = 1 };
enum class Phase : std::uint8_t {
    kIdle = 0,
    kSettle,
    kTransfer,
    kLift,
    kHold,
    kLower,
    kTouchdown,
};

enum class Preset : std::uint8_t { kConservative = 0, kVisualization = 1 };

struct RockingParameters {
    // Signed targets in the common robot frame.  A stance target moves the
    // torso over the loaded foot; a swing target tucks the unloaded foot
    // toward the centerline.  Keeping them separate avoids asking the loaded
    // and unloaded hip-roll servos to track the same angle.
    float left_stance_roll_rad = -1.5f * DEG2RAD;
    float right_stance_roll_rad = +1.5f * DEG2RAD;
    float left_swing_roll_rad = -8.0f * DEG2RAD;
    float right_swing_roll_rad = +8.0f * DEG2RAD;
    float lift_hip_delta_rad = -12.0f * DEG2RAD;
    float lift_knee_delta_rad = +12.0f * DEG2RAD;

    std::uint32_t initial_settle_ms = 1000;
    std::uint32_t transfer_ms = 320;
    std::uint32_t lift_ms = 220;
    std::uint32_t hold_ms = 50;
    std::uint32_t lower_ms = 220;
    std::uint32_t touchdown_ms = 80;
};

struct RockingOutput {
    JointArray position_rad{};
    Phase phase = Phase::kIdle;
    Side support_side = Side::kLeft;
    std::uint64_t completed_events = 0;
    bool running = false;
};

RockingParameters parametersForPreset(Preset preset);
const char* phaseName(Phase phase);
const char* sideName(Side side);

class RockingGait {
public:
    explicit RockingGait(
        const RockingParameters& parameters = parametersForPreset(
            Preset::kConservative));

    void setParameters(const RockingParameters& parameters);
    const RockingParameters& parameters() const { return parameters_; }

    void reset();
    void start(Side first_support = Side::kLeft);
    void stop();
    RockingOutput update(std::uint32_t dt_ms);
    const RockingOutput& output() const { return output_; }

    std::uint32_t eventDurationMs() const;
    std::uint32_t cycleDurationMs() const { return 2u * eventDurationMs(); }

private:
    RockingOutput sampleEvent(std::uint32_t event_time_ms) const;
    void setStandOutput(Phase phase, bool running);

    RockingParameters parameters_{};
    RockingOutput output_{};
    Side support_side_ = Side::kLeft;
    std::uint32_t settle_elapsed_ms_ = 0;
    std::uint32_t event_elapsed_ms_ = 0;
    std::uint64_t completed_events_ = 0;
    bool running_ = false;
};

}  // namespace tilt::gait
