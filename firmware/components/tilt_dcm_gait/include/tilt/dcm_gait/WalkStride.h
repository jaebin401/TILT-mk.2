#pragma once

// Fore-aft foot placement on top of the stepping pattern (open-loop rhythm):
// turns rocking in place into walking.
//
// Each leg's sole is shifted by x (mm, +forward) in the torso frame and the
// shift is converted to joint angles with the leg IK at the stand sole pitch
// (the ankle keeps the sole parallel; delta from the stand pose, so the lift
// fold and the IMU feedback still add on top):
//   swing  : x = +s/2 (stepped with the lift: the foot lands ahead)
//   stance : x goes linearly from +s/2 to -s/2 over the stance time
//            (2 * period - hold), which carries the torso forward.
// s (the "stride", foot sweep per step relative to the hip) ramps toward the
// target at ramp_mm_s so a run always starts in place.
//
// Note (2026-10-09): the FLASH fold itself (hip -lift, knee +lift) moves the
// swing sole ~16 mm forward at 15 deg (and back at touchdown while the servos
// are still moving), which is the forward drift seen while "rocking in place".
// A small negative stride cancels it.
// Platform-neutral: no ESP-IDF dependencies.

#include <array>
#include <cstdint>

#include "tilt/dcm_gait/DcmStepping.h"

namespace tilt::dcm_gait {

struct WalkParams {
    // + forward, - backward. -2 mm = rocking in place on the 2026-10-09 floor
    // (cancels the forward drift of the lift fold).
    float stride_target_mm = -2.0f;
    float stride_in_place_mm = -2.0f;  // Target set at every run start.
    float stride_max_mm = 30.0f;
    float ramp_mm_s = 4.0f;  // Stride change rate (start / stop / changes).
};

class WalkStride {
public:
    explicit WalkStride(const WalkParams& params = WalkParams{});

    WalkParams& params() { return params_; }
    const WalkParams& params() const { return params_; }

    // Stride and both feet back to 0 (stand). The target is kept.
    void reset();

    // Once per control cycle after DcmStepping::update().
    void update(const Command& gait, const Params& gait_params, std::uint32_t dt_ms);

    // Adds the IK deltas for both legs (no-op at stride 0).
    void apply(JointArray& q) const;

    float stride_mm() const { return stride_mm_; }
    float foot_x_mm(int leg) const { return x_mm_[leg]; }

private:
    WalkParams params_{};
    float stride_mm_ = 0.0f;
    std::array<float, 2> x_mm_{};
    std::array<std::uint32_t, 2> stance_ms_{};
    std::array<float, NUM_LEG_JOINTS> stand_theta_[2]{};
    std::array<float, 3> stand_sole_[2]{};
    std::array<float, 2> stand_sole_pitch_{};
};

}  // namespace tilt::dcm_gait
