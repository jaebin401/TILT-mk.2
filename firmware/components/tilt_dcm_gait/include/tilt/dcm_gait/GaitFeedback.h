#pragma once

// IMU feedback layered on top of the stepping pattern (DcmStepping), mainly for
// the open-loop FLASH rocking on roll-rounded soles.
//
//   Layer 1  posture trim   (per gait cycle, slow integrator)
//            cycle-mean roll  -> common hip roll offset  (target 0 deg)
//            cycle-mean pitch -> common hip pitch offset (target pitch_target_deg)
//            Compensates floor slope / static asymmetry.
//   Layer 2  amplitude      (per step)
//            peak roll toward the stance side during the half cycle that
//            starts with a swing -> that side's extra lift (target amp_target_deg).
//            Keeps left/right rocking symmetric.
//   Layer 3  damping        (every control cycle)
//            low-pass gyro roll / pitch rate -> common hip roll / pitch offset.
//            Optional; servo dead time (40-60 ms) limits the usable gain.
//   Layer 0  phase sync     (open-loop lift timing)
//            rocking phase angle theta = atan2(-rate/omega, lean) of the body
//            toward the next swing side; the lift is moved (within
//            +-lift_hint_window_ms of the timer) so it happens at the phase
//            learned from steady rocking. Hardware 2026-10-09: steady lifts
//            happen while the body is already rolling back from that side;
//            the falls came from lifts at the wrong phase.
//   Start ramp: the first ramp_steps swings use a reduced lift.
//
// Sign conventions. URDF, both legs: hip roll axis +x, hip pitch axis +y. With
// the feet holding their orientation on the floor, a common joint change dq
// rotates the torso by -dq about the same axis (measured on hardware
// 2026-10-09: +hip roll trim drove the mean IMU roll negative). IMU: roll + =
// left side up (leaning right), pitch + = nose down (leaning forward). So the
// trim is dq = +gain * (measured - target) for both roll and pitch.
// Platform-neutral: no ESP-IDF dependencies.

#include <array>
#include <cstdint>

#include "tilt/dcm_gait/DcmStepping.h"

namespace tilt::dcm_gait {

struct FeedbackParams {
    bool trim_enabled = true;    // Layer 1 (roll part).
    bool pitch_trim_enabled = true;  // Layer 1 pitch part (needs trim_enabled); sign verified 2026-10-09.
    bool amp_enabled = true;     // Layer 2.
    bool damp_enabled = false;   // Layer 3.
    bool phase_sync_enabled = false;  // Layer 0.
    std::uint32_t ramp_steps = 0;  // 0 = no start ramp. 3 broke the rocking phase (2026-10-09).

    // Layer 1: trim += gain * error once per gait cycle (deg per deg).
    float roll_trim_gain = 0.3f;
    float pitch_trim_gain = 0.3f;
    // 2.5 deg was the flat-floor rocking mean; 1.0 deg (torso a bit further
    // back) stopped the forward start-up falls on 2026-10-09 evening.
    float pitch_target_deg = 1.0f;
    float trim_step_max_deg = 0.5f;    // Max change per cycle.
    float roll_trim_max_deg = 4.0f;
    float pitch_trim_max_deg = 5.0f;
    float trim_slew_deg_s = 10.0f;     // Applied offset follows the trim at this rate.
    float trim_learn_max_err_deg = 5.0f;  // No learning when the cycle mean is off by more.

    // Layer 2: lift_delta += gain * (target - excursion) once per step. The
    // excursion is measured from the previous cycle-mean roll (so a slope or a
    // trim offset does not look like an amplitude error).
    float amp_target_deg = 7.0f;       // Flat-floor rocking: 6.6 (R) / 7.1 (L).
    float amp_gain = 0.15f;
    float amp_max_deg = 3.0f;
    // Learned lift deltas are faded in over this many cycles of each run:
    // starting a run directly with +3 deg (lift 18) pitched the robot forward
    // and it fell within 2 cycles (2026-10-09).
    std::uint32_t lift_fade_cycles = 8;

    // Layer 3: offset = gain * filtered rate (deg per deg/s).
    float damp_roll_gain = 0.0f;
    float damp_pitch_gain = 0.03f;
    float damp_max_deg = 3.0f;
    float gyro_tau_s = 0.02f;

    // Layer 0: phase plane (lean, -rate/omega), omega = 2 pi / 0.6 s.
    float phase_omega_rad_s = 10.5f;
    std::uint32_t phase_learn_start_cycle = 10;  // Learn after the rhythm settles.
    std::uint32_t phase_learn_samples = 8;       // Lifts per side to average.
    // Learn only from a settled rhythm: the last 4 excursions of each side
    // within this spread and the cycle-mean roll within phase_learn_max_mean_deg
    // (a run on 2026-10-09 learned during a period-2 rhythm and averaged the
    // two lift phases into one that fitted neither).
    float phase_learn_spread_deg = 2.0f;
    float phase_learn_max_mean_deg = 1.0f;
    float phase_min_amp_deg = 1.5f;   // Below this the phase is meaningless.
    float phase_accept_deg = 150.0f;  // Lift when 0 <= theta - nominal < this.

    // Layers 1 and 2 do not learn from a window whose |roll| or |pitch| peak
    // exceeds this (stumble / push), nor during the start ramp.
    float freeze_tilt_deg = 12.0f;
};

struct FeedbackState {
    float roll_trim_deg = 0.0f;     // Learned (layer 1).
    float pitch_trim_deg = 0.0f;
    float applied_roll_deg = 0.0f;  // Slew-limited value actually applied.
    float applied_pitch_deg = 0.0f;
    std::array<float, 2> lift_delta_deg{};  // [left, right] learned (layer 2).
    std::array<float, 2> excursion_deg{};   // Last measured, per swing side.
    float damp_roll_deg = 0.0f;     // Layer 3 output.
    float damp_pitch_deg = 0.0f;
    float cycle_roll_deg = 0.0f;    // Last completed cycle means.
    float cycle_pitch_deg = 0.0f;
    std::uint32_t cycles = 0;       // Completed gait cycles this run.
    bool cycle_done = false;        // True on the update a cycle completed.
    bool last_window_frozen = false;
    bool rhythm_stable = false;  // Layer 0 learning gate.
    bool roll_trim_saturated = false;   // At the limit and still off: check the sign.
    std::array<float, 2> phase_nominal_deg{};  // Learned lift phase per swing side.
    std::array<bool, 2> phase_learned{};
    float lift_phase_deg = 0.0f;   // Phase at the last lift (swing side).
    bool pitch_trim_saturated = false;
};

// Where apply() is called from: the posture offsets are used in all three,
// damping and lift only while stepping.
enum class FeedbackContext : std::uint8_t { kStand = 0, kStepping = 1 };

class GaitFeedback {
public:
    explicit GaitFeedback(const FeedbackParams& params = FeedbackParams{});

    void setParams(const FeedbackParams& params) { params_ = params; }
    const FeedbackParams& params() const { return params_; }
    FeedbackParams& mutableParams() { return params_; }
    const FeedbackState& state() const { return state_; }

    // Forgets everything learned (trims, lift deltas).
    void reset();
    // Call when a stepping run starts: clears the windows, keeps what was learned.
    void beginRun();
    // Call when a run ends in a fall (tilt stop): drops what was learned since
    // the last cycle with a settled rhythm (the run-up to a fall teaches junk,
    // e.g. lift L +3 / R +0.4 after a crash, which then broke the next start).
    void revertToLastGood();

    // Once per control cycle, after DcmStepping::update(). Angles in rad,
    // rates in rad/s.
    void update(const Command& gait, bool stepping, float roll_rad, float pitch_rad,
                float roll_rate_rad_s, float pitch_rate_rad_s, float dt_s);

    // Adds the feedback to a joint command. base_lift_deg is the pattern's lift
    // (Params::lift_deg); scale in [0, 1] fades the posture offsets in (stand
    // transition).
    // lift_mode: the pattern's LiftMode, so the extra lift folds the same way.
    void apply(JointArray& q, const Command& gait, FeedbackContext context,
               float base_lift_deg, float scale = 1.0f,
               LiftMode lift_mode = LiftMode::kFlash) const;

    // Extra lift (deg, may be negative) for the swing side: ramp + layer 2.
    float extraLiftDeg(const Command& gait, float base_lift_deg) const;

    // Layer 0: lift timing hint for DcmStepping::setLiftHint (open loop).
    LiftHint liftHint(const Command& gait) const;
    // Rocking phase (deg) toward `side` and its amplitude (deg), from the last update.
    float phaseDeg(Side side, float* amplitude_deg = nullptr) const;

private:
    void closeWindow();

    FeedbackParams params_{};
    FeedbackState state_{};

    // Current half-cycle window (from one swing start to the next).
    bool window_open_ = false;
    Side window_side_ = Side::kRight;
    std::uint32_t window_step_ = 0;
    float window_min_roll_ = 0.0f;
    float window_max_roll_ = 0.0f;
    float window_peak_tilt_ = 0.0f;
    // Full-cycle accumulators (two windows).
    double cycle_roll_sum_ = 0.0;
    double cycle_pitch_sum_ = 0.0;
    std::uint32_t cycle_samples_ = 0;
    int cycle_windows_ = 0;
    bool cycle_frozen_ = false;

    float roll_rate_lpf_ = 0.0f;
    // Last values learned while the rhythm was settled (see revertToLastGood).
    float good_roll_trim_ = 0.0f;
    float good_pitch_trim_ = 0.0f;
    std::array<float, 2> good_lift_{};
    float last_roll_deg_ = 0.0f;
    std::array<double, 2> phase_cos_sum_{};
    std::array<double, 2> phase_sin_sum_{};
    std::array<std::uint32_t, 2> phase_count_{};
    std::array<std::array<float, 4>, 2> recent_excursion_{};
    std::array<std::uint32_t, 2> recent_count_{};
    float pitch_rate_lpf_ = 0.0f;
};

}  // namespace tilt::dcm_gait
