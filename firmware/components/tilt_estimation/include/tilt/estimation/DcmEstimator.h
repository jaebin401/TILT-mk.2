#pragma once

// Divergent Component of Motion (DCM) estimator for TILT mk.2.
//
//   xi = c + c_dot / omega,   omega = sqrt(g / z_c)
//
// Inputs every control cycle: commanded leg joint angles, torso attitude and
// gyro from the IMU, and which foot is on the ground. Outputs are expressed in
// a gravity-aligned ground frame G whose origin is the reference sole point
// (stance sole, or the midpoint of both soles in double support). G has the
// torso's heading (yaw is not estimated) and z up. Units: mm, mm/s, rad.
//
// Assumptions: the reference foot does not slip; in single support the body
// rotates about the foot edge that is on the ground (or the sole centre when
// the foot is flat). Joint angles may be commanded values (with the optional
// first-order lag model) or servo read-back values (set joint_lag_s = 0).
// Read-back is preferred: under load the stance hip roll deflects ~3 deg in
// simulation, which biases the CoM by ~7 mm when commanded angles are used.
// Platform-neutral: no ESP-IDF dependencies.

#include <cstdint>

#include "tilt/estimation/CenterOfMass.h"
#include "tilt_config.h"
#include "tilt_kinematics.h"

namespace tilt::estimation {

enum class Support : std::uint8_t { kDouble = 0, kLeft = 1, kRight = 2 };

struct ImuSample {
    // Torso attitude in the robot convention (right-hand rule):
    //   roll  = rotation about +x (positive: left side up, torso tilts right)
    //   pitch = rotation about +y (positive: nose down)
    float roll_rad = 0.0f;
    float pitch_rad = 0.0f;
    // Body-frame angular rate about +x, +y, +z.
    float gyro_rad_s[3] = {0.0f, 0.0f, 0.0f};
    bool valid = false;
};

struct EstimatorConfig {
    float gravity_mm_s2 = 9806.65f;
    // First-order model of servo tracking applied to the commanded angles.
    // 0 disables it (uses commanded angles directly).
    float joint_lag_s = 0.03f;
    // CoM velocity = gyro term (body rotation about the pivot, unfiltered)
    //             + joint term (CoM motion relative to the stance foot from
    //               joint angle changes, low-passed because differentiating
    //               encoder readings amplifies their noise).
    // Low-pass time constant of the joint term. 0 disables filtering.
    float joint_velocity_filter_s = 0.03f;
    // Optional low-pass on the total velocity (adds lag; 0 = off).
    float velocity_filter_s = 0.0f;
    float min_height_mm = 50.0f;
    // Foot geometry around the sole point (MJCF sole proxy: 70 x 38 mm).
    float foot_half_width_mm = 19.0f;
    float foot_half_length_mm = 35.0f;
    // In single support the body rotates about the foot edge that touches the
    // ground. The pivot moves from the sole centre to that edge as the foot's
    // roll (in G) grows to this angle.
    float pivot_roll_full_rad = 0.5f * DEG2RAD;
    // IMU mounting signs (+1 / -1), to be verified on hardware.
    float roll_sign = 1.0f;
    float pitch_sign = 1.0f;
    float gyro_sign[3] = {1.0f, 1.0f, 1.0f};
    UpperBodyPose upper{};
};

struct DcmState {
    bool valid = false;
    Support support = Support::kDouble;
    Vec3 com_mm{};             // CoM relative to the reference sole point, in G.
    Vec3 com_velocity_mm_s{};  // In G.
    float omega_rad_s = 0.0f;
    float height_mm = 0.0f;    // CoM height above the reference sole point.
    float dcm_mm[2] = {0.0f, 0.0f};  // xi (x, y) in G.
    Vec3 sole_mm[2]{};         // Both sole points in G (0 = LEFT, 1 = RIGHT).
    // How far the DCM has passed each foot's inner edge toward that foot,
    // laterally (mm). >= 0: the DCM is on or beyond the inner edge (over the
    // foot or outside it); < 0: still between the feet.
    float dcm_past_inner_edge_mm[2] = {0.0f, 0.0f};
    // Lateral DCM distance beyond each foot's outer edge (> 0 = outside the foot).
    float dcm_past_outer_edge_mm[2] = {0.0f, 0.0f};
    Vec3 com_body_mm{};        // CoM in base frame B (diagnostics).
    float stance_foot_roll_rad = 0.0f;  // Roll of the support foot in G.
};

class DcmEstimator {
public:
    explicit DcmEstimator(const EstimatorConfig& config = EstimatorConfig{});

    void setConfig(const EstimatorConfig& config);
    const EstimatorConfig& config() const { return config_; }

    // Re-initialises the joint model and filters at the given joint angles.
    void reset(const float leg_theta[NUM_JOINTS]);

    // One control cycle. Returns the updated state (also kept in state()).
    const DcmState& update(const float commanded_theta[NUM_JOINTS],
                           const ImuSample& imu, Support support, float dt_s);

    const DcmState& state() const { return state_; }
    // Joint angles after the servo-lag model (diagnostics).
    const float* modelledTheta() const { return theta_; }

private:
    EstimatorConfig config_{};
    DcmState state_{};
    float theta_[NUM_JOINTS]{};
    Vec3 com_body_prev_{};
    Vec3 sole_body_prev_[2]{};
    Vec3 velocity_filtered_{};
    Vec3 joint_velocity_filtered_{};
    bool initialised_ = false;
};

}  // namespace tilt::estimation
