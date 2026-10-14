#include "tilt/dcm_gait/WalkStride.h"

#include <algorithm>
#include <cmath>

#include "tilt_kinematics.h"

namespace tilt::dcm_gait {
namespace {

Leg toLeg(int i) { return i == 0 ? Leg::LEFT : Leg::RIGHT; }

}  // namespace

WalkStride::WalkStride(const WalkParams& params) : params_(params) {
    const JointArray stand = standPose();
    for (int i = 0; i < 2; ++i) {
        float theta[NUM_LEG_JOINTS];
        for (int j = 0; j < NUM_LEG_JOINTS; ++j) theta[j] = stand[i * NUM_LEG_JOINTS + j];
        const Vec3 p = fk_foot(toLeg(i), theta);
        stand_sole_[i] = {p.x, p.y, p.z};
        stand_sole_pitch_[i] = sole_pitch_rad(theta);
        // Same branch as the deltas below.
        const IkResult r = ik_foot(toLeg(i), p, stand_sole_pitch_[i]);
        for (int j = 0; j < NUM_LEG_JOINTS; ++j) stand_theta_[i][j] = r.theta[j];
    }
}

void WalkStride::reset() {
    stride_mm_ = 0.0f;
    x_mm_ = {};
    stance_ms_ = {};
}

void WalkStride::update(const Command& gait, const Params& gait_params, std::uint32_t dt_ms) {
    if (!gait.running) {
        reset();
        return;
    }
    const float target = std::clamp(params_.stride_target_mm, -params_.stride_max_mm,
                                    params_.stride_max_mm);
    const float step = params_.ramp_mm_s * static_cast<float>(dt_ms) / 1000.0f;
    stride_mm_ += std::clamp(target - stride_mm_, -step, step);

    const std::uint32_t period = gait_params.open_period_ms;
    const std::uint32_t hold = gait_params.open_hold_ms;
    const float stance_time = static_cast<float>(2 * period > hold ? 2 * period - hold : period);
    for (int i = 0; i < 2; ++i) {
        const bool swinging = gait.phase == Phase::kSwing &&
                              (gait.swing == Side::kLeft ? 0 : 1) == i;
        if (swinging) {
            stance_ms_[i] = 0;
            x_mm_[i] = 0.5f * stride_mm_;
        } else {
            stance_ms_[i] += dt_ms;
            const float frac = std::min(1.0f, static_cast<float>(stance_ms_[i]) / stance_time);
            x_mm_[i] = stride_mm_ * (0.5f - frac);
        }
    }
}

void WalkStride::apply(JointArray& q) const {
    for (int i = 0; i < 2; ++i) {
        if (std::fabs(x_mm_[i]) < 1e-3f) continue;
        const Vec3 target{stand_sole_[i][0] + x_mm_[i], stand_sole_[i][1], stand_sole_[i][2]};
        const IkResult r = ik_foot(toLeg(i), target, stand_sole_pitch_[i]);
        if (!r.reachable) continue;
        for (int j = 0; j < NUM_LEG_JOINTS; ++j) {
            q[i * NUM_LEG_JOINTS + j] += r.theta[j] - stand_theta_[i][j];
        }
    }
}

}  // namespace tilt::dcm_gait
