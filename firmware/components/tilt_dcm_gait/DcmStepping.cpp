#include "tilt/dcm_gait/DcmStepping.h"

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
    return {0.0f, -20.0f * DEG2RAD, 40.0f * DEG2RAD,
            0.0f, -20.0f * DEG2RAD, 40.0f * DEG2RAD};
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
        case Trigger::kOpenLoopTimer: return "timer";
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

void DcmStepping::start() {
    command_ = Command{};
    command_.running = true;
    first_swing_ = true;
    next_swing_ = params_.first_swing;
    enterSwing(next_swing_, Trigger::kStart);
}

void DcmStepping::stop(StopReason reason) {
    if (!command_.running) return;
    finish(reason);
}

void DcmStepping::finish(StopReason reason) {
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
    command_.support = stanceSupport(swing);
    command_.phase_ms = 0;
    command_.last_trigger = trigger;
    command_.phase_changed = true;
    writePose();
}

void DcmStepping::enterDouble(Trigger trigger) {
    ++command_.steps;
    first_swing_ = false;
    next_swing_ = opposite(command_.swing);
    if (params_.max_steps > 0 && command_.steps >= params_.max_steps) {
        finish(StopReason::kStepLimit);
        command_.last_trigger = trigger;
        return;
    }
    command_.phase = Phase::kDouble;
    command_.support = Support::kDouble;
    command_.phase_ms = 0;
    command_.last_trigger = trigger;
    command_.phase_changed = true;
    writePose();
}

void DcmStepping::writePose() {
    JointArray q = standPose();
    if (command_.phase == Phase::kSwing) {
        const int base = legIndex(command_.swing) * 3;
        const float lift = params_.lift_deg * DEG2RAD;
        q[base + 1] -= lift;  // Hip pitch forward.
        q[base + 2] += lift;  // Knee flexes: sole stays parallel.
    }
    command_.position_rad = q;
}

const Command& DcmStepping::update(const DcmState& state, std::uint32_t dt_ms) {
    command_.phase_changed = false;
    if (!command_.running) return command_;
    command_.phase_ms += dt_ms;
    const std::uint32_t t = command_.phase_ms;

    if (params_.mode == Mode::kOpenLoop) {
        if (command_.phase == Phase::kSwing && t >= params_.open_hold_ms) {
            enterDouble(Trigger::kOpenLoopTimer);
        } else if (command_.phase == Phase::kDouble) {
            const std::uint32_t gap = params_.open_period_ms > params_.open_hold_ms
                ? params_.open_period_ms - params_.open_hold_ms
                : 0;
            if (t >= gap) enterSwing(next_swing_, Trigger::kOpenLoopTimer);
        }
        return command_;
    }

    if (command_.phase == Phase::kSwing) {
        const int stance = legIndex(opposite(command_.swing));
        if (state.dcm_past_outer_edge_mm[stance] > params_.abort_outer_mm) {
            finish(StopReason::kOuterEdge);
            return command_;
        }
        if (first_swing_) {
            if (t >= params_.start_hold_ms) enterDouble(Trigger::kOpenLoopTimer);
            return command_;
        }
        if (t >= params_.swing_max_ms) {
            enterDouble(Trigger::kSwingTimeout);
        } else if (t >= params_.swing_min_ms &&
                   dcmInsideSwingFoot(state, command_.swing) <=
                       params_.touchdown_dcm_inside_mm) {
            enterDouble(Trigger::kDcmTouchdown);
        }
        return command_;
    }

    if (command_.phase == Phase::kDouble) {
        // The leg that just landed becomes the stance leg for the next swing.
        const int stance = legIndex(opposite(next_swing_));
        if (t >= params_.double_max_ms) {
            finish(StopReason::kDoubleTimeout);
        } else if (t >= params_.double_min_ms &&
                   state.dcm_past_inner_edge_mm[stance] >=
                       params_.lift_dcm_past_inner_mm) {
            enterSwing(next_swing_, Trigger::kDcmLift);
        }
    }
    return command_;
}

}  // namespace tilt::dcm_gait
