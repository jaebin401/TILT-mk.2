#pragma once

// DCM-gated stepping in place (lateral rocking) for TILT mk.2.
//
// Each swing is a FLASH lift: the swing leg's hip pitch and knee are stepped
// by -lift / +lift at once (no interpolation), which keeps the sole parallel.
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
//               the gait stops.
//   Open-loop mode: fixed hold / period (FLASH baseline for comparison).
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

enum class StopReason : std::uint8_t {
    kNone = 0,
    kOperator,
    kStepLimit,
    kDoubleTimeout,
    kOuterEdge,
};

struct Params {
    Mode mode = Mode::kDcm;
    float lift_deg = 15.0f;
    Side first_swing = Side::kRight;
    std::uint32_t max_steps = 0;  // 0 = unlimited.

    // DCM mode.
    std::uint32_t start_hold_ms = 90;
    std::uint32_t swing_min_ms = 60;
    std::uint32_t swing_max_ms = 160;
    float touchdown_dcm_inside_mm = 21.0f;  // LIPM b_y at T = 0.2 s.
    std::uint32_t double_min_ms = 20;
    std::uint32_t double_max_ms = 300;
    float lift_dcm_past_inner_mm = 0.0f;
    float abort_outer_mm = 15.0f;

    // Open-loop mode (FLASH).
    std::uint32_t open_hold_ms = 90;
    std::uint32_t open_period_ms = 200;
};

// Why the last phase change happened (for logging).
enum class Trigger : std::uint8_t {
    kNone = 0,
    kStart,
    kDcmTouchdown,   // DCM reached the touchdown target.
    kSwingTimeout,   // swing_max_ms reached.
    kDcmLift,        // DCM passed the stance foot's inner edge.
    kOpenLoopTimer,
    kStop,
};

struct Command {
    JointArray position_rad{};
    Phase phase = Phase::kIdle;
    Support support = Support::kDouble;  // For the estimator.
    Side swing = Side::kRight;           // Valid in kSwing.
    std::uint32_t steps = 0;             // Completed touchdowns.
    std::uint32_t phase_ms = 0;          // Time in the current phase.
    Trigger last_trigger = Trigger::kNone;
    StopReason stop_reason = StopReason::kNone;
    bool running = false;
    bool phase_changed = false;  // True on the cycle a phase starts.
};

JointArray standPose();
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

    void start();
    void stop(StopReason reason = StopReason::kOperator);
    bool running() const { return command_.running; }

    // Call once per control cycle with the latest estimator state.
    const Command& update(const DcmState& state, std::uint32_t dt_ms);
    const Command& command() const { return command_; }

private:
    void enterSwing(Side swing, Trigger trigger);
    void enterDouble(Trigger trigger);
    void finish(StopReason reason);
    void writePose();

    Params params_{};
    Command command_{};
    Side next_swing_ = Side::kRight;
    bool first_swing_ = true;
};

}  // namespace tilt::dcm_gait
