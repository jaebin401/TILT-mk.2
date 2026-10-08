#include "tilt/gait/RockingGait.h"

#include <algorithm>
#include <cmath>

namespace tilt::gait {
namespace {

constexpr float kPi = 3.14159265358979323846f;

JointArray standPose() {
    return {
        0.0f,
        -20.0f * DEG2RAD,
        +40.0f * DEG2RAD,
        0.0f,
        -20.0f * DEG2RAD,
        +40.0f * DEG2RAD,
    };
}

Side opposite(Side side) {
    return side == Side::kLeft ? Side::kRight : Side::kLeft;
}

}  // namespace

RockingParameters parametersForPreset(Preset preset) {
    RockingParameters parameters;
    if (preset == Preset::kVisualization) {
        parameters.left_stance_roll_rad = -2.0f * DEG2RAD;
        parameters.right_stance_roll_rad = +2.0f * DEG2RAD;
        parameters.left_swing_roll_rad = -10.0f * DEG2RAD;
        parameters.right_swing_roll_rad = +10.0f * DEG2RAD;
        parameters.lift_hip_delta_rad = -16.0f * DEG2RAD;
        parameters.lift_knee_delta_rad = +16.0f * DEG2RAD;
        parameters.initial_settle_ms = 1000;
        parameters.half_cycle_ms = 460;
        parameters.build_rock_halves = 4;
        parameters.lift_ramp_halves = 2;
    }
    return parameters;
}

const char* phaseName(Phase phase) {
    switch (phase) {
        case Phase::kIdle: return "IDLE";
        case Phase::kSettle: return "SETTLE";
        case Phase::kTransfer: return "TRANSFER";
        case Phase::kLift: return "LIFT";
        case Phase::kHold: return "HOLD";
        case Phase::kLower: return "LOWER";
        case Phase::kTouchdown: return "TOUCHDOWN";
    }
    return "UNKNOWN";
}

const char* sideName(Side side) {
    return side == Side::kLeft ? "LEFT" : "RIGHT";
}

const char* hipRollModeName(HipRollMode mode) {
    switch (mode) {
        case HipRollMode::kPreset: return "PRESET";
        case HipRollMode::kDisabled: return "PITCH_ONLY";
        case HipRollMode::kCustom: return "CUSTOM";
    }
    return "UNKNOWN";
}

RockingGait::RockingGait(const RockingParameters& parameters)
    : parameters_(parameters) {
    reset();
}

void RockingGait::setParameters(const RockingParameters& parameters) {
    parameters_ = parameters;
    reset();
}

void RockingGait::reset() {
    running_ = false;
    support_side_ = Side::kLeft;
    settle_elapsed_ms_ = 0;
    event_elapsed_ms_ = 0;
    completed_events_ = 0;
    setStandOutput(Phase::kIdle, false);
}

void RockingGait::start(Side first_support) {
    support_side_ = first_support;
    settle_elapsed_ms_ = 0;
    event_elapsed_ms_ = 0;
    completed_events_ = 0;
    running_ = true;
    setStandOutput(parameters_.initial_settle_ms > 0 ? Phase::kSettle
                                                     : Phase::kTransfer,
                   true);
}

void RockingGait::stop() {
    running_ = false;
    setStandOutput(Phase::kIdle, false);
}

std::uint32_t RockingGait::eventDurationMs() const {
    return parameters_.half_cycle_ms;
}

void RockingGait::setStandOutput(Phase phase, bool running) {
    output_.position_rad = standPose();
    output_.phase = phase;
    output_.support_side = support_side_;
    output_.completed_events = completed_events_;
    output_.running = running;
}

RockingOutput RockingGait::sampleEvent(std::uint32_t event_time_ms) const {
    RockingOutput result;
    result.position_rad = standPose();
    result.support_side = support_side_;
    result.completed_events = completed_events_;
    result.running = true;

    const float phase = parameters_.half_cycle_ms == 0
        ? 0.0f
        : std::clamp(static_cast<float>(event_time_ms) /
                         static_cast<float>(parameters_.half_cycle_ms),
                     0.0f, 1.0f);
    // A half sine has no finite DSP window: the old swing foot reaches zero
    // exactly when the next swing foot starts moving.  At the boundary both
    // targets are zero for one mathematical instant, but the oscillator does
    // not stop or restart with zero velocity.
    const float wave = std::sin(kPi * phase);

    float lift_gain = 0.0f;
    const std::uint32_t build_halves =
        parameters_.hip_roll_mode == HipRollMode::kDisabled
            ? 0u : parameters_.build_rock_halves;
    if (completed_events_ >= build_halves) {
        if (parameters_.lift_ramp_halves == 0) {
            lift_gain = 1.0f;
        } else {
            const std::uint64_t ramp_half =
                completed_events_ - build_halves + 1;
            lift_gain = std::min(
                1.0f,
                static_cast<float>(ramp_half) /
                    static_cast<float>(parameters_.lift_ramp_halves));
        }
    }
    const float lift_scale = lift_gain * wave;

    if (lift_gain == 0.0f) {
        result.phase = Phase::kTransfer;
    } else if (phase < 0.5f) {
        result.phase = Phase::kLift;
    } else {
        result.phase = Phase::kLower;
    }

    float left_roll = 0.0f;
    float right_roll = 0.0f;
    if (parameters_.hip_roll_mode == HipRollMode::kPreset) {
        if (support_side_ == Side::kLeft) {
            left_roll = parameters_.left_stance_roll_rad * wave;
            right_roll = parameters_.right_swing_roll_rad * lift_gain * wave;
        } else {
            left_roll = parameters_.left_swing_roll_rad * lift_gain * wave;
            right_roll = parameters_.right_stance_roll_rad * wave;
        }
    } else if (parameters_.hip_roll_mode == HipRollMode::kCustom) {
        if (support_side_ == Side::kLeft) {
            left_roll = parameters_.custom_left_support_left_roll_rad * wave;
            right_roll = parameters_.custom_left_support_right_roll_rad * wave;
        } else {
            left_roll =
                -parameters_.custom_left_support_right_roll_rad * wave;
            right_roll =
                -parameters_.custom_left_support_left_roll_rad * wave;
        }
    }
    result.position_rad[L_HIP_ROLL] = left_roll;
    result.position_rad[R_HIP_ROLL] = right_roll;

    const int swing_base =
        support_side_ == Side::kLeft ? R_HIP_ROLL : L_HIP_ROLL;
    result.position_rad[swing_base + 1] +=
        parameters_.lift_hip_delta_rad * lift_scale;
    result.position_rad[swing_base + 2] +=
        parameters_.lift_knee_delta_rad * lift_scale;
    return result;
}

RockingOutput RockingGait::update(std::uint32_t dt_ms) {
    if (!running_) {
        setStandOutput(Phase::kIdle, false);
        return output_;
    }

    std::uint32_t remaining = dt_ms;
    if (settle_elapsed_ms_ < parameters_.initial_settle_ms) {
        const std::uint32_t settle_remaining =
            parameters_.initial_settle_ms - settle_elapsed_ms_;
        const std::uint32_t advance = std::min(remaining, settle_remaining);
        settle_elapsed_ms_ += advance;
        remaining -= advance;
        if (settle_elapsed_ms_ < parameters_.initial_settle_ms ||
            remaining == 0) {
            setStandOutput(Phase::kSettle, true);
            return output_;
        }
    }

    const std::uint32_t duration = eventDurationMs();
    while (remaining > 0 && duration > 0) {
        const std::uint32_t event_remaining = duration - event_elapsed_ms_;
        const std::uint32_t advance = std::min(remaining, event_remaining);
        event_elapsed_ms_ += advance;
        remaining -= advance;
        if (event_elapsed_ms_ >= duration) {
            event_elapsed_ms_ = 0;
            ++completed_events_;
            support_side_ = opposite(support_side_);
        }
    }

    output_ = sampleEvent(event_elapsed_ms_);
    output_.completed_events = completed_events_;
    return output_;
}

}  // namespace tilt::gait
