#include "tilt/gait/RockingGait.h"

#include <algorithm>

namespace tilt::gait {
namespace {

float smoothstep5(float value) {
    const float s = std::clamp(value, 0.0f, 1.0f);
    return s * s * s * (10.0f + s * (-15.0f + 6.0f * s));
}

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

float lerp(float from, float to, float scale) {
    return from + (to - from) * scale;
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
        parameters.left_swing_roll_rad = -12.0f * DEG2RAD;
        parameters.right_swing_roll_rad = +12.0f * DEG2RAD;
        parameters.lift_hip_delta_rad = -14.0f * DEG2RAD;
        parameters.lift_knee_delta_rad = +14.0f * DEG2RAD;
        parameters.initial_settle_ms = 1000;
        parameters.transfer_ms = 350;
        parameters.lift_ms = 220;
        parameters.hold_ms = 30;
        parameters.lower_ms = 220;
        parameters.touchdown_ms = 40;
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
    return parameters_.transfer_ms + parameters_.lift_ms +
           parameters_.hold_ms + parameters_.lower_ms +
           parameters_.touchdown_ms;
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

    const std::uint32_t lift_start = parameters_.transfer_ms;
    const std::uint32_t lift_top = lift_start + parameters_.lift_ms;
    const std::uint32_t lift_down = lift_top + parameters_.hold_ms;
    const std::uint32_t lift_end = lift_down + parameters_.lower_ms;

    const bool first_event = completed_events_ == 0;
    const Side previous_support = opposite(support_side_);

    // At touchdown the previous swing hip has returned to neutral while the
    // previous stance hip is still carrying the torso.  The next transfer
    // blends directly from that pose to the new stance pose; it never returns
    // both hips to stand and never inserts a stationary DSP phase.
    float transfer_from_left = 0.0f;
    float transfer_from_right = 0.0f;
    if (!first_event) {
        if (previous_support == Side::kLeft) {
            transfer_from_left = parameters_.left_stance_roll_rad;
        } else {
            transfer_from_right = parameters_.right_stance_roll_rad;
        }
    }
    const float transfer_to_left = support_side_ == Side::kLeft
        ? parameters_.left_stance_roll_rad
        : 0.0f;
    const float transfer_to_right = support_side_ == Side::kRight
        ? parameters_.right_stance_roll_rad
        : 0.0f;

    float left_roll = transfer_to_left;
    float right_roll = transfer_to_right;
    if (event_time_ms < lift_start && parameters_.transfer_ms > 0) {
        const float transfer_scale = smoothstep5(
            static_cast<float>(event_time_ms) / parameters_.transfer_ms);
        left_roll = lerp(transfer_from_left, transfer_to_left, transfer_scale);
        right_roll = lerp(transfer_from_right, transfer_to_right, transfer_scale);
    }

    float lift_scale = 0.0f;
    if (event_time_ms < lift_start) {
        result.phase = Phase::kTransfer;
    } else if (event_time_ms < lift_top && parameters_.lift_ms > 0) {
        result.phase = Phase::kLift;
        lift_scale = smoothstep5(
            static_cast<float>(event_time_ms - lift_start) /
            parameters_.lift_ms);
    } else if (event_time_ms < lift_down) {
        result.phase = Phase::kHold;
        lift_scale = 1.0f;
    } else if (event_time_ms < lift_end && parameters_.lower_ms > 0) {
        result.phase = Phase::kLower;
        lift_scale = 1.0f - smoothstep5(
            static_cast<float>(event_time_ms - lift_down) /
            parameters_.lower_ms);
    } else {
        result.phase = Phase::kTouchdown;
    }

    if (support_side_ == Side::kLeft) {
        right_roll += parameters_.right_swing_roll_rad * lift_scale;
    } else {
        left_roll += parameters_.left_swing_roll_rad * lift_scale;
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
