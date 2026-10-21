#include "tilt/dcm_gait/DcmStepping.h"

#include <algorithm>

namespace tilt::dcm_gait {
namespace {

Side opposite(Side side) {
    return side == Side::kLeft ? Side::kRight : Side::kLeft;
}

Support stanceSupport(Side swing) {
    return swing == Side::kLeft ? Support::kRight : Support::kLeft;
}

int legIndex(Side side) { return side == Side::kLeft ? 0 : 1; }

}  // namespace

JointArray standPose() {
    JointArray q{};
    for (int j = 0; j < NUM_JOINTS; ++j) q[j] = ZERO_POSE_RAD[j];
    return q;
}

void addLift(JointArray& q, Side swing, float lift_rad, LiftMode mode) {
    const int base = legIndex(swing) * NUM_LEG_JOINTS;
    if (mode == LiftMode::kVertical) {
        q[base + 1] -= lift_rad;         // Thigh forward.
        q[base + 2] += 2.0f * lift_rad;  // Shank back by the same angle.
        q[base + 3] -= lift_rad;         // Sole stays parallel.
    } else {
        q[base + 1] -= lift_rad;  // Hip pitch forward.
        q[base + 2] += lift_rad;  // Knee flexes: sole stays parallel.
    }
}

float smoothLiftProgress(std::uint32_t elapsed_ms, std::uint32_t duration_ms) {
    if (duration_ms == 0) return 1.0f;
    const float s = std::clamp(static_cast<float>(elapsed_ms) /
                                   static_cast<float>(duration_ms),
                               0.0f, 1.0f);
    return s * s * s * (10.0f + s * (-15.0f + 6.0f * s));
}

const char* liftModeName(LiftMode mode) {
    return mode == LiftMode::kVertical ? "vertical" : "flash";
}

const char* phaseName(Phase phase) {
    switch (phase) {
        case Phase::kIdle: return "IDLE";
        case Phase::kSwing: return "SWING";
        case Phase::kDouble: return "DOUBLE";
    }
    return "?";
}

const char* triggerName(Trigger trigger) {
    switch (trigger) {
        case Trigger::kNone: return "-";
        case Trigger::kStart: return "start";
        case Trigger::kDcmTouchdown: return "dcm_touchdown";
        case Trigger::kSwingTimeout: return "swing_timeout";
        case Trigger::kDcmLift: return "dcm_lift";
        case Trigger::kDoubleTimeout: return "double_timeout_lift";
        case Trigger::kOpenLoopTimer: return "timer";
        case Trigger::kPhaseSync: return "phase_sync";
        case Trigger::kStop: return "stop";
    }
    return "?";
}

const char* stopReasonName(StopReason reason) {
    switch (reason) {
        case StopReason::kNone: return "none";
        case StopReason::kOperator: return "operator";
        case StopReason::kStepLimit: return "step_limit";
        case StopReason::kDoubleTimeout: return "double_timeout";
        case StopReason::kOuterEdge: return "outer_edge";
        case StopReason::kTilt: return "tilt";
    }
    return "?";
}

float dcmInsideSwingFoot(const DcmState& state, Side swing) {
    const int i = legIndex(swing);
    const float rel = state.dcm_mm[1] - state.sole_mm[i].y;
    // Left foot: midline is -y of it. Right foot: midline is +y of it.
    return swing == Side::kLeft ? -rel : rel;
}

DcmStepping::DcmStepping(const Params& params) : params_(params) {
    command_.position_rad = standPose();
}

void DcmStepping::start(const DcmState* state) {
    command_ = Command{};
    command_.running = true;
    first_swing_ = true;
    stop_after_landing_ = false;
    landing_active_ = false;
    consecutive_fallbacks_ = 0;
    next_swing_ = params_.first_swing;
    if (params_.auto_first_swing && state != nullptr && state->valid) {
        // In double support the DCM is measured from the midpoint of the soles.
        const float y = state->dcm_mm[1];
        if (y > params_.auto_first_swing_deadband_mm) next_swing_ = Side::kLeft;
        if (y < -params_.auto_first_swing_deadband_mm) next_swing_ = Side::kRight;
    }
    enterSwing(next_swing_, Trigger::kStart);
}

void DcmStepping::stop(StopReason reason) {
    if (!command_.running) return;
    finish(reason);
}

void DcmStepping::finish(StopReason reason) {
    landing_active_ = false;
    command_.running = false;
    command_.phase = Phase::kIdle;
    command_.support = Support::kDouble;
    command_.stop_reason = reason;
    command_.last_trigger = Trigger::kStop;
    command_.phase_ms = 0;
    command_.phase_changed = true;
    writePose();
}

void DcmStepping::enterSwing(Side swing, Trigger trigger) {
    command_.phase = Phase::kSwing;
    command_.swing = swing;
    next_swing_ = opposite(swing);
    command_.support = stanceSupport(swing);
    command_.phase_ms = 0;
    command_.last_trigger = trigger;
    command_.phase_changed = true;
    writePose();
}

void DcmStepping::enterDouble(Trigger trigger) {
    landing_active_ = false;
    ++command_.steps;
    first_swing_ = false;
    next_swing_ = opposite(command_.swing);
    command_.phase = Phase::kDouble;
    command_.support = Support::kDouble;
    command_.phase_ms = 0;
    command_.last_trigger = trigger;
    command_.phase_changed = true;
    stop_after_landing_ = params_.max_steps > 0 && command_.steps >= params_.max_steps;
    writePose();
    // Preserve legacy immediate stop when no landing trajectory was requested.
    if (stop_after_landing_ && params_.lift_fall_ms == 0) finish(StopReason::kStepLimit);
}

void DcmStepping::enterOpenLoopSwing(Side swing, Trigger trigger) {
    ++command_.steps;
    first_swing_ = false;
    landing_side_ = command_.swing;
    landing_active_ = params_.lift_fall_ms > 0;
    enterSwing(swing, trigger);
}

void DcmStepping::writePose() {
    JointArray q = standPose();
    std::array<float, 2> scale{};
    if (command_.phase == Phase::kSwing) {
        scale[legIndex(command_.swing)] =
            smoothLiftProgress(command_.phase_ms, params_.lift_rise_ms);
        if (params_.mode == Mode::kOpenLoop && params_.open_crossfade && landing_active_) {
            scale[legIndex(landing_side_)] =
                1.0f - smoothLiftProgress(command_.phase_ms, params_.lift_fall_ms);
        }
    } else if (command_.phase == Phase::kDouble && params_.lift_fall_ms > 0) {
        // command_.swing still names the leg that has just touched down.
        scale[legIndex(command_.swing)] =
            1.0f - smoothLiftProgress(command_.phase_ms, params_.lift_fall_ms);
    }
    for (const Side side : {Side::kLeft, Side::kRight}) {
        const float leg_scale = scale[legIndex(side)];
        if (leg_scale > 0.0f) {
            addLift(q, side, params_.lift_deg * DEG2RAD * leg_scale, params_.lift_mode);
        }
    }
    command_.lift_scale_by_leg = scale;
    command_.lift_scale = scale[legIndex(command_.swing)];
    command_.position_rad = q;
}

const Command& DcmStepping::update(const DcmState& state, std::uint32_t dt_ms) {
    command_.phase_changed = false;
    if (!command_.running) return command_;
    command_.phase_ms += dt_ms;
    const std::uint32_t t = command_.phase_ms;

    // A finite step count lands through the same synchronized trajectory before
    // becoming idle. This is especially important for the single-lift commands.
    if (command_.phase == Phase::kDouble && stop_after_landing_) {
        if (t >= params_.lift_fall_ms) {
            finish(StopReason::kStepLimit);
        } else {
            writePose();
        }
        return command_;
    }

    if (params_.mode == Mode::kOpenLoop) {
        if (!params_.open_crossfade) {
            const std::uint32_t swing_ms =
                std::max(params_.open_hold_ms, params_.lift_rise_ms);
            if (command_.phase == Phase::kSwing && t >= swing_ms) {
                enterDouble(Trigger::kOpenLoopTimer);
            } else if (command_.phase == Phase::kDouble) {
                const std::uint32_t gap = params_.open_period_ms > swing_ms
                    ? params_.open_period_ms - swing_ms
                    : 0;
                const std::uint32_t w = params_.lift_hint_window_ms;
                const std::uint32_t early = gap > w ? gap - w : 0;
                if (lift_hint_ == LiftHint::kNone) {
                    if (t >= std::max(gap, params_.lift_fall_ms)) {
                        enterSwing(next_swing_, Trigger::kOpenLoopTimer);
                    }
                } else if (lift_hint_ == LiftHint::kLift &&
                           t >= std::max(early, params_.lift_fall_ms)) {
                    enterSwing(next_swing_, Trigger::kPhaseSync);
                } else if (t >= std::max(gap + w, params_.lift_fall_ms)) {
                    enterSwing(next_swing_, Trigger::kOpenLoopTimer);
                }
            }
            writePose();
            return command_;
        }
        if (command_.phase == Phase::kSwing) {
            const bool final_swing = params_.max_steps > 0 &&
                                     command_.steps + 1 >= params_.max_steps;
            if (final_swing) {
                const std::uint32_t hold =
                    std::max(params_.open_hold_ms, params_.lift_rise_ms);
                if (t >= hold) enterDouble(Trigger::kOpenLoopTimer);
            } else {
                // A crossfade cannot start before the current rise and the
                // preceding landing can both finish inside one half-cycle.
                const std::uint32_t period = std::max(
                    params_.open_period_ms,
                    std::max(params_.lift_rise_ms, params_.lift_fall_ms));
                const std::uint32_t w = params_.lift_hint_window_ms;
                const std::uint32_t earliest = std::max(
                    period > w ? period - w : 0,
                    std::max(params_.lift_rise_ms, params_.lift_fall_ms));
                if (lift_hint_ == LiftHint::kLift && t >= earliest) {
                    enterOpenLoopSwing(next_swing_, Trigger::kPhaseSync);
                } else if (lift_hint_ == LiftHint::kNone && t >= period) {
                    enterOpenLoopSwing(next_swing_, Trigger::kOpenLoopTimer);
                } else if (lift_hint_ == LiftHint::kWait && t >= period + w) {
                    enterOpenLoopSwing(next_swing_, Trigger::kOpenLoopTimer);
                }
            }
        }
        writePose();
        return command_;
    }

    if (command_.phase == Phase::kSwing) {
        const int stance = legIndex(opposite(command_.swing));
        if (state.dcm_past_outer_edge_mm[stance] > params_.abort_outer_mm) {
            finish(StopReason::kOuterEdge);
            return command_;
        }
        if (first_swing_) {
            if (t >= std::max(params_.start_hold_ms, params_.lift_rise_ms)) {
                enterDouble(Trigger::kOpenLoopTimer);
            }
            writePose();
            return command_;
        }
        const std::uint32_t swing_min = std::max(params_.swing_min_ms, params_.lift_rise_ms);
        const std::uint32_t swing_max = std::max(params_.swing_max_ms, params_.lift_rise_ms);
        if (t >= swing_max) {
            enterDouble(Trigger::kSwingTimeout);
        } else if (t >= swing_min &&
                   dcmInsideSwingFoot(state, command_.swing) <=
                       params_.touchdown_dcm_inside_mm) {
            enterDouble(Trigger::kDcmTouchdown);
        }
        writePose();
        return command_;
    }

    if (command_.phase == Phase::kDouble) {
        // The leg that just landed becomes the stance leg for the next swing.
        const int stance = legIndex(opposite(next_swing_));
        const std::uint32_t double_min = std::max(params_.double_min_ms, params_.lift_fall_ms);
        const std::uint32_t double_max = std::max(params_.double_max_ms, params_.lift_fall_ms);
        if (t >= double_min &&
            state.dcm_past_inner_edge_mm[stance] >= params_.lift_dcm_past_inner_mm) {
            consecutive_fallbacks_ = 0;
            enterSwing(next_swing_, Trigger::kDcmLift);
        } else if (t >= double_max) {
            if (consecutive_fallbacks_ >= params_.max_double_fallbacks) {
                finish(StopReason::kDoubleTimeout);
            } else {
                ++consecutive_fallbacks_;
                enterSwing(next_swing_, Trigger::kDoubleTimeout);
            }
        }
    }
    writePose();
    return command_;
}

}  // namespace tilt::dcm_gait
