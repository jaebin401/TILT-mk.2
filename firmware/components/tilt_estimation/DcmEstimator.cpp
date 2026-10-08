#include "tilt/estimation/DcmEstimator.h"

#include <cmath>

namespace tilt::estimation {
namespace {

Vec3 sub(const Vec3& a, const Vec3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
Vec3 add(const Vec3& a, const Vec3& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
Vec3 scale(const Vec3& a, float s) { return {a.x * s, a.y * s, a.z * s}; }
Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

Vec3 rotate(const Mat3& R, const Vec3& v) {
    return {
        R.m[0][0] * v.x + R.m[0][1] * v.y + R.m[0][2] * v.z,
        R.m[1][0] * v.x + R.m[1][1] * v.y + R.m[1][2] * v.z,
        R.m[2][0] * v.x + R.m[2][1] * v.y + R.m[2][2] * v.z,
    };
}

// Torso orientation in G: R = Ry(pitch) * Rx(roll) (yaw ignored).
Mat3 attitude(float roll, float pitch) {
    const float cr = std::cos(roll);
    const float sr = std::sin(roll);
    const float cp = std::cos(pitch);
    const float sp = std::sin(pitch);
    return {{
        {cp, sp * sr, sp * cr},
        {0.0f, cr, -sr},
        {-sp, cp * sr, cp * cr},
    }};
}

Vec3 reference_point(Support support, const Vec3 sole[2]) {
    switch (support) {
        case Support::kLeft: return sole[0];
        case Support::kRight: return sole[1];
        case Support::kDouble:
        default: return scale(add(sole[0], sole[1]), 0.5f);
    }
}

float blend(float dt, float tau) {
    return tau > 0.0f ? dt / (tau + dt) : 1.0f;
}

}  // namespace

DcmEstimator::DcmEstimator(const EstimatorConfig& config) : config_(config) {}

void DcmEstimator::setConfig(const EstimatorConfig& config) {
    config_ = config;
}

void DcmEstimator::reset(const float leg_theta[NUM_JOINTS]) {
    for (int j = 0; j < NUM_JOINTS; ++j) theta_[j] = leg_theta[j];
    com_body_prev_ = center_of_mass(theta_, config_.upper);
    sole_body_prev_[0] = fk_foot(Leg::LEFT, &theta_[0]);
    sole_body_prev_[1] = fk_foot(Leg::RIGHT, &theta_[3]);
    velocity_filtered_ = {0.0f, 0.0f, 0.0f};
    joint_velocity_filtered_ = {0.0f, 0.0f, 0.0f};
    state_ = DcmState{};
    initialised_ = true;
}

const DcmState& DcmEstimator::update(const float commanded_theta[NUM_JOINTS],
                                     const ImuSample& imu, Support support,
                                     float dt_s) {
    if (!initialised_) reset(commanded_theta);
    if (!(dt_s > 0.0f)) dt_s = 1.0e-3f;

    // 1. Servo tracking model.
    const float a = blend(dt_s, config_.joint_lag_s);
    for (int j = 0; j < NUM_JOINTS; ++j) {
        theta_[j] += (commanded_theta[j] - theta_[j]) * a;
    }

    // 2. Kinematics in B.
    const Vec3 com_body = center_of_mass(theta_, config_.upper);
    const LegFrames frames[2] = {fk_leg(Leg::LEFT, &theta_[0]),
                                 fk_leg(Leg::RIGHT, &theta_[3])};
    const Vec3 sole_body[2] = {frames[0].sole.p, frames[1].sole.p};
    const Vec3 ref_body = reference_point(support, sole_body);
    const Vec3 ref_body_prev = reference_point(support, sole_body_prev_);

    // 3. Orientation of B in G.
    const float roll = config_.roll_sign * imu.roll_rad;
    const float pitch = config_.pitch_sign * imu.pitch_rad;
    const Mat3 R = imu.valid ? attitude(roll, pitch) : attitude(0.0f, 0.0f);

    // 4. CoM relative to the reference sole, gravity aligned.
    const Vec3 r_body = sub(com_body, ref_body);
    const Vec3 r = rotate(R, r_body);

    // 5. Pivot: in single support the robot rotates about the foot edge on
    //    the ground. Foot roll in G decides which edge (positive roll = the
    //    foot's +y edge is up, so the -y edge is the pivot).
    Vec3 pivot{0.0f, 0.0f, 0.0f};  // Relative to the reference point, in G.
    float foot_roll = 0.0f;
    if (support != Support::kDouble) {
        const int leg = support == Support::kLeft ? 0 : 1;
        const Mat3 sole_R = frames[leg].sole.R;
        // y axis of the sole frame expressed in G.
        const Vec3 sole_y = rotate(R, {sole_R.m[0][1], sole_R.m[1][1], sole_R.m[2][1]});
        foot_roll = std::atan2(sole_y.z, sole_y.y);
        float share = foot_roll / config_.pivot_roll_full_rad;
        if (share > 1.0f) share = 1.0f;
        if (share < -1.0f) share = -1.0f;
        pivot = scale(sole_y, -share * config_.foot_half_width_mm);
    }
    state_.stance_foot_roll_rad = foot_roll;

    // 6. Velocity: body rotation about the pivot + joint motion.
    //    The reference point uses the same support for both samples so a
    //    support switch does not create a jump.
    const Vec3 relative_motion = scale(
        sub(sub(com_body, com_body_prev_), sub(ref_body, ref_body_prev)),
        1.0f / dt_s);
    Vec3 omega_world{0.0f, 0.0f, 0.0f};
    if (imu.valid) {
        const Vec3 gyro_body{config_.gyro_sign[0] * imu.gyro_rad_s[0],
                             config_.gyro_sign[1] * imu.gyro_rad_s[1],
                             config_.gyro_sign[2] * imu.gyro_rad_s[2]};
        omega_world = rotate(R, gyro_body);
    }
    const float bj = blend(dt_s, config_.joint_velocity_filter_s);
    joint_velocity_filtered_ = add(
        joint_velocity_filtered_,
        scale(sub(rotate(R, relative_motion), joint_velocity_filtered_), bj));
    const Vec3 velocity_raw =
        add(cross(omega_world, sub(r, pivot)), joint_velocity_filtered_);
    const float b = blend(dt_s, config_.velocity_filter_s);
    velocity_filtered_ = add(velocity_filtered_,
                             scale(sub(velocity_raw, velocity_filtered_), b));

    // 7. Linear inverted pendulum frequency and DCM.
    float height = r.z;
    if (height < config_.min_height_mm) height = config_.min_height_mm;
    const float omega = std::sqrt(config_.gravity_mm_s2 / height);

    state_.valid = imu.valid;
    state_.support = support;
    state_.com_mm = r;
    state_.com_velocity_mm_s = velocity_filtered_;
    state_.omega_rad_s = omega;
    state_.height_mm = r.z;
    state_.dcm_mm[0] = r.x + velocity_filtered_.x / omega;
    state_.dcm_mm[1] = r.y + velocity_filtered_.y / omega;
    state_.com_body_mm = com_body;

    // 8. Feet in G and lateral edge margins.
    const float hw = config_.foot_half_width_mm;
    for (int leg = 0; leg < 2; ++leg) {
        state_.sole_mm[leg] = rotate(R, sub(sole_body[leg], ref_body));
        const float rel = state_.dcm_mm[1] - state_.sole_mm[leg].y;
        // Left foot: inner side is -y. Right foot: inner side is +y.
        if (leg == 0) {
            state_.dcm_past_inner_edge_mm[0] = rel + hw;
            state_.dcm_past_outer_edge_mm[0] = rel - hw;
        } else {
            state_.dcm_past_inner_edge_mm[1] = hw - rel;
            state_.dcm_past_outer_edge_mm[1] = -rel - hw;
        }
    }

    com_body_prev_ = com_body;
    sole_body_prev_[0] = sole_body[0];
    sole_body_prev_[1] = sole_body[1];
    return state_;
}

}  // namespace tilt::estimation
