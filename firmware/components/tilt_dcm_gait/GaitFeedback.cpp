#include "tilt/dcm_gait/GaitFeedback.h"

#include <algorithm>
#include <cmath>

namespace tilt::dcm_gait {
namespace {

constexpr float kRadToDeg = 57.2957795f;

int legIndex(Side side) { return side == Side::kLeft ? 0 : 1; }

float clampAbs(float value, float limit) { return std::clamp(value, -limit, limit); }

float slew(float current, float target, float max_step) {
    return current + clampAbs(target - current, max_step);
}

float wrap180(float deg) {
    while (deg > 180.0f) deg -= 360.0f;
    while (deg <= -180.0f) deg += 360.0f;
    return deg;
}

Side opposite(Side s) { return s == Side::kLeft ? Side::kRight : Side::kLeft; }

}  // namespace

GaitFeedback::GaitFeedback(const FeedbackParams& params) : params_(params) {}

void GaitFeedback::reset() {
    state_ = FeedbackState{};
    good_roll_trim_ = 0.0f;
    good_pitch_trim_ = 0.0f;
    good_lift_ = {};
    phase_cos_sum_ = {};
    phase_sin_sum_ = {};
    phase_count_ = {};
    recent_count_ = {};
    beginRun();
    roll_rate_lpf_ = 0.0f;
    pitch_rate_lpf_ = 0.0f;
}

void GaitFeedback::beginRun() {
    window_open_ = false;
    recent_count_ = {};
    state_.rhythm_stable = false;
    cycle_roll_sum_ = 0.0;
    cycle_pitch_sum_ = 0.0;
    cycle_samples_ = 0;
    cycle_windows_ = 0;
    cycle_frozen_ = false;
    state_.cycles = 0;
    state_.cycle_done = false;
    // The previous run's cycle mean is not this run's centre (a fall leaves
    // e.g. +8 deg here, which skewed the first excursions).
    state_.cycle_roll_deg = 0.0f;
    state_.cycle_pitch_deg = 0.0f;
    state_.damp_roll_deg = 0.0f;
    state_.damp_pitch_deg = 0.0f;
}

void GaitFeedback::revertToLastGood() {
    state_.roll_trim_deg = good_roll_trim_;
    state_.pitch_trim_deg = good_pitch_trim_;
    state_.lift_delta_deg = good_lift_;
}

void GaitFeedback::closeWindow() {
    window_open_ = false;
    const int i = legIndex(window_side_);
    // Swinging the right leg rocks the body onto the left foot (roll < 0).
    const float centre = state_.cycles > 0 ? state_.cycle_roll_deg : 0.0f;
    const float excursion = window_side_ == Side::kRight ? centre - window_min_roll_
                                                         : window_max_roll_ - centre;
    state_.excursion_deg[i] = excursion;
    recent_excursion_[i][recent_count_[i] % 4] = excursion;
    ++recent_count_[i];
    const bool frozen = window_peak_tilt_ > params_.freeze_tilt_deg ||
                        window_step_ < params_.ramp_steps;
    state_.last_window_frozen = frozen;
    if (params_.amp_enabled && !frozen) {
        state_.lift_delta_deg[i] = clampAbs(
            state_.lift_delta_deg[i] + params_.amp_gain * (params_.amp_target_deg - excursion),
            params_.amp_max_deg);
    }

    cycle_frozen_ = cycle_frozen_ || frozen;
    if (++cycle_windows_ < 2) return;
    {
        bool stable = !cycle_frozen_;
        for (int s = 0; s < 2; ++s) {
            if (recent_count_[s] < 4) {
                stable = false;
                continue;
            }
            const auto& r = recent_excursion_[s];
            const float spread = *std::max_element(r.begin(), r.end()) -
                                 *std::min_element(r.begin(), r.end());
            stable = stable && spread <= params_.phase_learn_spread_deg;
        }
        state_.rhythm_stable = stable;  // Mean-roll check added below.
    }

    // One full gait cycle (a left and a right half) completed.
    if (cycle_samples_ > 0) {
        state_.cycle_roll_deg = static_cast<float>(cycle_roll_sum_ / cycle_samples_);
        state_.cycle_pitch_deg = static_cast<float>(cycle_pitch_sum_ / cycle_samples_);
        ++state_.cycles;
        state_.cycle_done = true;
        state_.rhythm_stable = state_.rhythm_stable &&
                               std::fabs(state_.cycle_roll_deg) <= params_.phase_learn_max_mean_deg;
        if (state_.rhythm_stable) {
            good_roll_trim_ = state_.roll_trim_deg;
            good_pitch_trim_ = state_.pitch_trim_deg;
            good_lift_ = state_.lift_delta_deg;
        }
        // dq = -d(torso angle): leaning left (roll < 0) -> -hip roll rotates the
        // torso back up; leaning forward past the target -> +hip pitch.
        const float roll_err = state_.cycle_roll_deg;
        const float pitch_err = state_.cycle_pitch_deg - params_.pitch_target_deg;
        if (params_.trim_enabled && !cycle_frozen_ &&
            std::fabs(roll_err) < params_.trim_learn_max_err_deg) {
            const float d_roll = clampAbs(params_.roll_trim_gain * roll_err, params_.trim_step_max_deg);
            state_.roll_trim_deg =
                clampAbs(state_.roll_trim_deg + d_roll, params_.roll_trim_max_deg);
        }
        if (params_.trim_enabled && params_.pitch_trim_enabled && !cycle_frozen_ &&
            std::fabs(pitch_err) < params_.trim_learn_max_err_deg) {
            const float d_pitch = clampAbs(params_.pitch_trim_gain * pitch_err, params_.trim_step_max_deg);
            state_.pitch_trim_deg =
                clampAbs(state_.pitch_trim_deg + d_pitch, params_.pitch_trim_max_deg);
        }
        state_.roll_trim_saturated =
            std::fabs(state_.roll_trim_deg) >= params_.roll_trim_max_deg - 1e-3f &&
            std::fabs(roll_err) > 1.0f;
        state_.pitch_trim_saturated =
            std::fabs(state_.pitch_trim_deg) >= params_.pitch_trim_max_deg - 1e-3f &&
            std::fabs(pitch_err) > 1.0f;
    }
    cycle_roll_sum_ = 0.0;
    cycle_pitch_sum_ = 0.0;
    cycle_samples_ = 0;
    cycle_windows_ = 0;
    cycle_frozen_ = false;
}

void GaitFeedback::update(const Command& gait, bool stepping, float roll_rad, float pitch_rad,
                          float roll_rate_rad_s, float pitch_rate_rad_s, float dt_s) {
    state_.cycle_done = false;
    const float roll = roll_rad * kRadToDeg;
    const float pitch = pitch_rad * kRadToDeg;
    last_roll_deg_ = roll;

    // Rate filter runs always so it is settled when stepping starts.
    const float a = dt_s / (params_.gyro_tau_s + dt_s);
    roll_rate_lpf_ += a * (roll_rate_rad_s * kRadToDeg - roll_rate_lpf_);
    pitch_rate_lpf_ += a * (pitch_rate_rad_s * kRadToDeg - pitch_rate_lpf_);

    // Posture offsets follow the learned trim (or 0 when layer 1 is off).
    const float step = params_.trim_slew_deg_s * dt_s;
    state_.applied_roll_deg = slew(state_.applied_roll_deg,
                                   params_.trim_enabled ? state_.roll_trim_deg : 0.0f, step);
    state_.applied_pitch_deg = slew(
        state_.applied_pitch_deg,
        params_.trim_enabled && params_.pitch_trim_enabled ? state_.pitch_trim_deg : 0.0f, step);

    if (!stepping || !gait.running) {
        window_open_ = false;
        cycle_windows_ = 0;
        cycle_samples_ = 0;
        cycle_roll_sum_ = cycle_pitch_sum_ = 0.0;
        cycle_frozen_ = false;
        state_.damp_roll_deg = 0.0f;
        state_.damp_pitch_deg = 0.0f;
        return;
    }

    if (gait.phase_changed && gait.phase == Phase::kSwing) {
        // Layer 0 learning: phase of this lift toward the lifted side.
        const int i = legIndex(gait.swing);
        float amp = 0.0f;
        const float theta = phaseDeg(gait.swing, &amp);
        state_.lift_phase_deg = theta;
        if (!state_.phase_learned[i] && state_.cycles >= params_.phase_learn_start_cycle &&
            state_.rhythm_stable && !state_.last_window_frozen &&
            amp >= params_.phase_min_amp_deg) {
            phase_cos_sum_[i] += std::cos(theta * DEG2RAD);
            phase_sin_sum_[i] += std::sin(theta * DEG2RAD);
            if (++phase_count_[i] >= params_.phase_learn_samples) {
                state_.phase_nominal_deg[i] = static_cast<float>(
                    std::atan2(phase_sin_sum_[i], phase_cos_sum_[i]) * kRadToDeg);
                state_.phase_learned[i] = true;
            }
        }
        if (window_open_) closeWindow();
        window_open_ = true;
        window_side_ = gait.swing;
        window_step_ = gait.steps;
        window_min_roll_ = roll;
        window_max_roll_ = roll;
        window_peak_tilt_ = 0.0f;
    }
    if (window_open_) {
        window_min_roll_ = std::min(window_min_roll_, roll);
        window_max_roll_ = std::max(window_max_roll_, roll);
        window_peak_tilt_ = std::max(window_peak_tilt_, std::max(std::fabs(roll), std::fabs(pitch)));
        cycle_roll_sum_ += roll;
        cycle_pitch_sum_ += pitch;
        ++cycle_samples_;
    }

    if (params_.damp_enabled) {
        // Same sign rule as the trim: rolling right (rate > 0) -> +hip roll.
        state_.damp_roll_deg = clampAbs(params_.damp_roll_gain * roll_rate_lpf_, params_.damp_max_deg);
        // Pitching forward (rate > 0): pitch the torso back (+hip pitch).
        state_.damp_pitch_deg = clampAbs(params_.damp_pitch_gain * pitch_rate_lpf_, params_.damp_max_deg);
    } else {
        state_.damp_roll_deg = 0.0f;
        state_.damp_pitch_deg = 0.0f;
    }
}

float GaitFeedback::phaseDeg(Side side, float* amplitude_deg) const {
    const float sign = side == Side::kRight ? 1.0f : -1.0f;  // roll + = leaning right.
    const float lean = sign * (last_roll_deg_ - state_.cycle_roll_deg);
    const float away = -sign * roll_rate_lpf_ / params_.phase_omega_rad_s;  // deg
    if (amplitude_deg != nullptr) *amplitude_deg = std::sqrt(lean * lean + away * away);
    return std::atan2(away, lean) * kRadToDeg;
}

LiftHint GaitFeedback::liftHint(const Command& gait) const {
    if (!params_.phase_sync_enabled || !gait.running ||
        (gait.phase != Phase::kSwing && gait.phase != Phase::kDouble)) {
        return LiftHint::kNone;
    }
    const Side next = opposite(gait.swing);
    if (!state_.phase_learned[legIndex(next)]) return LiftHint::kNone;
    float amp = 0.0f;
    const float theta = phaseDeg(next, &amp);
    if (amp < params_.phase_min_amp_deg) return LiftHint::kNone;
    const float d = wrap180(theta - state_.phase_nominal_deg[legIndex(next)]);
    return d >= 0.0f && d < params_.phase_accept_deg ? LiftHint::kLift : LiftHint::kWait;
}

float GaitFeedback::extraLiftDeg(const Command& gait, float base_lift_deg) const {
    return extraLiftDegForSide(gait, gait.swing, base_lift_deg);
}

float GaitFeedback::extraLiftDegForSide(const Command& gait, Side side,
                                        float base_lift_deg) const {
    const int i = legIndex(side);
    const bool lifting = gait.phase == Phase::kSwing && side == gait.swing;
    const bool landing = !lifting && gait.lift_scale_by_leg[i] > 0.0f;
    if (!lifting && !landing) return 0.0f;
    // A touchdown/crossfade increments steps before lowering the previous leg,
    // so keep using that leg's preceding swing index while it descends.
    const std::uint32_t swing_index = landing && gait.steps > 0 ? gait.steps - 1 : gait.steps;
    float extra = 0.0f;
    if (params_.ramp_steps > 0 && swing_index < params_.ramp_steps) {
        // Swing k (k = completed touchdowns) of the ramp: lift * (k+1)/(n+1).
        const float scale = static_cast<float>(swing_index + 1) /
                            static_cast<float>(params_.ramp_steps + 1);
        extra = base_lift_deg * (scale - 1.0f);
    } else if (params_.amp_enabled) {
        float fade = 1.0f;
        if (params_.lift_fade_cycles > 0) {
            fade = std::min(1.0f, static_cast<float>(state_.cycles) /
                                      static_cast<float>(params_.lift_fade_cycles));
        }
        extra = fade * state_.lift_delta_deg[i];
    }
    return std::max(extra, -base_lift_deg);
}

void GaitFeedback::apply(JointArray& q, const Command& gait, FeedbackContext context,
                         float base_lift_deg, float scale, LiftMode lift_mode) const {
    const bool stepping = context == FeedbackContext::kStepping && gait.running;
    scale = std::clamp(scale, 0.0f, 1.0f);
    float roll = state_.applied_roll_deg * scale;
    float pitch = state_.applied_pitch_deg * scale;
    if (stepping) {
        roll += state_.damp_roll_deg;
        pitch += state_.damp_pitch_deg;
    }
    for (int leg = 0; leg < 2; ++leg) {
        q[leg * NUM_LEG_JOINTS + 0] += roll * DEG2RAD;   // Hip roll.
        q[leg * NUM_LEG_JOINTS + 1] += pitch * DEG2RAD;  // Hip pitch.
    }
    if (stepping && (gait.phase == Phase::kSwing || gait.phase == Phase::kDouble)) {
        // Scale each side independently: in an open-loop crossfade the new leg
        // rises while the previous leg keeps its continuous landing correction.
        for (const Side side : {Side::kLeft, Side::kRight}) {
            const float lift_scale = gait.lift_scale_by_leg[legIndex(side)];
            if (lift_scale <= 0.0f) continue;
            addLift(q, side,
                    extraLiftDegForSide(gait, side, base_lift_deg) *
                        lift_scale * DEG2RAD,
                    lift_mode);
        }
    }
}

}  // namespace tilt::dcm_gait
