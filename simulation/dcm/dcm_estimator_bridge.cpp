// C bridge exposing the firmware DCM estimator to Python (ctypes).
#include <new>

#include "tilt/estimation/DcmEstimator.h"

using tilt::estimation::DcmEstimator;
using tilt::estimation::EstimatorConfig;
using tilt::estimation::ImuSample;
using tilt::estimation::Support;

extern "C" {

void* tilt_dcm_create(float joint_lag_s, float joint_velocity_filter_s) {
    EstimatorConfig config;
    config.joint_lag_s = joint_lag_s;
    config.joint_velocity_filter_s = joint_velocity_filter_s;
    return new (std::nothrow) DcmEstimator(config);
}

void tilt_dcm_destroy(void* handle) { delete static_cast<DcmEstimator*>(handle); }

void tilt_dcm_reset(void* handle, const float theta[6]) {
    if (handle != nullptr) static_cast<DcmEstimator*>(handle)->reset(theta);
}

// out[18]: com xyz, com velocity xyz, omega, height, dcm xy,
//          past_inner L R, past_outer L R, sole L xy, sole R xy
int tilt_dcm_update(void* handle, const float theta[6], float roll, float pitch,
                    const float gyro[3], int imu_valid, int support, float dt,
                    float out[18]) {
    if (handle == nullptr || out == nullptr) return 0;
    ImuSample imu;
    imu.roll_rad = roll;
    imu.pitch_rad = pitch;
    for (int i = 0; i < 3; ++i) imu.gyro_rad_s[i] = gyro[i];
    imu.valid = imu_valid != 0;
    const auto& s = static_cast<DcmEstimator*>(handle)->update(
        theta, imu, static_cast<Support>(support), dt);
    const float values[18] = {
        s.com_mm.x, s.com_mm.y, s.com_mm.z,
        s.com_velocity_mm_s.x, s.com_velocity_mm_s.y, s.com_velocity_mm_s.z,
        s.omega_rad_s, s.height_mm, s.dcm_mm[0], s.dcm_mm[1],
        s.dcm_past_inner_edge_mm[0], s.dcm_past_inner_edge_mm[1],
        s.dcm_past_outer_edge_mm[0], s.dcm_past_outer_edge_mm[1],
        s.sole_mm[0].x, s.sole_mm[0].y, s.sole_mm[1].x, s.sole_mm[1].y,
    };
    for (int i = 0; i < 18; ++i) out[i] = values[i];
    return 1;
}

}  // extern "C"
