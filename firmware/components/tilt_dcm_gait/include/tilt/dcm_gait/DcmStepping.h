#pragma once

// DCM-gated stepping in place (lateral rocking) for TILT mk.2.
//
// Each swing follows one shared lift trajectory for hip, knee, and ankle pitch.
// Scaling the complete fold by the same progress keeps the sole parallel during
// the whole motion, not only at the final pose. See LiftMode.
//
//   DCM mode (default)
//     start   : one open-loop lift of the first swing leg (start_hold_ms) to
//               create lateral momentum, then:
//     SWING   : touch down when the DCM has come back to within
//               touchdown_dcm_inside_mm of the swing foot centre (measured
//               toward the midline), but not before swing_min_ms and not
//               after swing_max_ms.
//     DOUBLE  : lift the other leg once the DCM has passed the new stance
//               foot's inner edge by lift_dcm_past_inner_mm (after
//               double_min_ms). If that does not happen within double_max_ms
//               the leg is lifted anyway (timer fallback); after
//               max_double_fallbacks consecutive fallbacks the gait stops.
//     first swing: with auto_first_swing, the leg on the side the DCM already
//               leans toward is lifted first (the lean then helps the body
//               fall onto that side's partner foot).
//   Open-loop mode: fixed hold / period by default. With open_crossfade, start
//               the opposite lift every open_period_ms while the previous leg
//               lands, so the cadence is not stretched by lift_fall_ms.
//
// Safety stops: DCM beyond the stance foot's outer edge by abort_outer_mm
// during a swing (falling outward), or the step limit.
// Platform-neutral: no ESP-IDF dependencies.

#include <array>
#include <cstdint>

#include "tilt/estimation/DcmEstimator.h"
#include "tilt_config.h"

namespace tilt::dcm_gait {

using JointArray = std::array<float, NUM_JOINTS>;
using estimation::DcmState;
using estimation::Support;

enum class Mode : std::uint8_t { kDcm = 0, kOpenLoop = 1 };
enum class Phase : std::uint8_t { kIdle = 0, kSwing = 1, kDouble = 2 };
enum class Side : std::uint8_t { kLeft = 0, kRight = 1 };

// How a lift of L folds the swing leg. Both keep the sole parallel.
//   kFlash    hip -L, knee +L,  ankle 0   : the mk.2 FLASH fold. At the stand pose
//             the sole moves ~1.0 mm/deg forward and ~0.5 mm/deg up.
//   kVertical hip -L, knee +2L, ankle -L  : thigh and shank fold symmetrically,
//             the sole goes straight up (~1.1 mm/deg at the stand pose).
enum class LiftMode : std::uint8_t { kFlash = 0, kVertical = 1 };

enum class StopReason : std::uint8_t {
    kNone = 0,
    kOperator,
    kStepLimit,
    kDoubleTimeout,
    kOuterEdge,
    kTilt,  // Set by the application (IMU tilt limit).
};

struct Params {
    Mode mode = Mode::kDcm;
    float lift_deg = 15.0f;
    LiftMode lift_mode = LiftMode::kFlash;
    // Quintic lift / landing trajectory. 0 preserves the legacy instantaneous
    // command for applications that have not opted in yet.
    std::uint32_t lift_rise_ms = 0;
    std::uint32_t lift_fall_ms = 0;
    Side first_swing = Side::kRight;
    std::uint32_t max_steps = 0;  // 0 = unlimited.

    // DCM mode.
    std::uint32_t start_hold_ms = 90;
    std::uint32_t swing_min_ms = 60;
    std::uint32_t swing_max_ms = 160;
    float touchdown_dcm_inside_mm = 21.0f;  // LIPM b_y at T = 0.2 s.
    std::uint32_t double_min_ms = 20;
    std::uint32_t double_max_ms = 300;
    std::uint32_t max_double_fallbacks = 3;  // 0 = stop at the first timeout.
    bool auto_first_swing = false;
    float auto_first_swing_deadband_mm = 1.0f;
    float lift_dcm_past_inner_mm = 0.0f;
    float abort_outer_mm = 15.0f;

    // Open-loop mode. `open_hold_ms` is the duration of a finite/single lift;
    // continuous rocking starts the opposite lift every `open_period_ms`.
    std::uint32_t open_hold_ms = 90;
    std::uint32_t open_period_ms = 200;
    bool open_crossfade = false;
    // Lift timing hint window (see setLiftHint): the lift may move up to this
    // much earlier / later than the open-loop timer.
    std::uint32_t lift_hint_window_ms = 60;
};

// Why the last phase change happened (for logging).
enum class Trigger : std::uint8_t {
    kNone = 0,
    kStart,
    kDcmTouchdown,   // DCM reached the touchdown target.
    kSwingTimeout,   // swing_max_ms reached.
    kDcmLift,        // DCM passed the stance foot's inner edge.
    kDoubleTimeout,  // double_max_ms reached, lifted anyway.
    kOpenLoopTimer,
    kPhaseSync,      // Open-loop lift moved by the lift hint (rocking phase).
    kStop,
};

// Lift timing hint for the next open-loop crossfade, from outside (IMU phase).
enum class LiftHint : std::int8_t { kNone = -1, kWait = 0, kLift = 1 };

struct Command {
    JointArray position_rad{};
    Phase phase = Phase::kIdle;
    Support support = Support::kDouble;  // For the estimator.
    Side swing = Side::kRight;           // Valid in kSwing.
    std::uint32_t steps = 0;             // Completed touchdowns.
    std::uint32_t phase_ms = 0;          // Time in the current phase.
    float lift_scale = 0.0f;             // Scale of `swing` (compatibility view).
    // Per-leg trajectory scales [left, right]. During an open-loop crossfade
    // both entries may be non-zero: one rises while the other lands.
    std::array<float, 2> lift_scale_by_leg{};
    Trigger last_trigger = Trigger::kNone;
    StopReason stop_reason = StopReason::kNone;
    bool running = false;
    bool phase_changed = false;  // True on the cycle a phase starts.
};

// tilt_config ZERO_POSE_RAD: [0, -20, +40, -20] deg per leg, sole flat.
JointArray standPose();
// Adds a lift of lift_rad (may be negative) to the swing leg of q.
void addLift(JointArray& q, Side swing, float lift_rad, LiftMode mode);
// Quintic 0..1 progress. A zero duration means an immediate completed motion.
float smoothLiftProgress(std::uint32_t elapsed_ms, std::uint32_t duration_ms);
const char* liftModeName(LiftMode mode);
const char* phaseName(Phase phase);
const char* triggerName(Trigger trigger);
const char* stopReasonName(StopReason reason);

// Lateral distance of the DCM from the swing foot centre, toward the midline
// (mm). Small or negative: the DCM is at or beyond the swing foot.
float dcmInsideSwingFoot(const DcmState& state, Side swing);

class DcmStepping {
public:
    explicit DcmStepping(const Params& params = Params{});

    void setParams(const Params& params) { params_ = params; }
    const Params& params() const { return params_; }

    // state: latest estimator output, used by auto_first_swing (may be null).
    void start(const DcmState* state = nullptr);
    void stop(StopReason reason = StopReason::kOperator);
    bool running() const { return command_.running; }

    // Open-loop mode: kNone keeps the timer; kLift crossfades as soon as the window
    // opens (timer - lift_hint_window_ms); kWait delays the lift up to
    // timer + lift_hint_window_ms. Set before every update().
    void setLiftHint(LiftHint hint) { lift_hint_ = hint; }

    // Call once per control cycle with the latest estimator state.
    const Command& update(const DcmState& state, std::uint32_t dt_ms);
    const Command& command() const { return command_; }

private:
    void enterSwing(Side swing, Trigger trigger);
    void enterDouble(Trigger trigger);
    void enterOpenLoopSwing(Side swing, Trigger trigger);
    void finish(StopReason reason);
    void writePose();

    Params params_{};
    Command command_{};
    Side next_swing_ = Side::kRight;
    bool first_swing_ = true;
    bool stop_after_landing_ = false;
    bool landing_active_ = false;
    Side landing_side_ = Side::kRight;
    std::uint32_t consecutive_fallbacks_ = 0;
    LiftHint lift_hint_ = LiftHint::kNone;
};

}  // namespace tilt::dcm_gait
