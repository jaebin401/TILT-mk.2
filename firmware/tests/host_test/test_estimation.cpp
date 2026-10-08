// Host tests for tilt_estimation (center of mass + DCM estimator).

#include <cmath>
#include <cstdio>

#include "kinematics_reference.h"
#include "tilt/estimation/CenterOfMass.h"
#include "tilt/estimation/DcmEstimator.h"

namespace {

using tilt::estimation::DcmEstimator;
using tilt::estimation::EstimatorConfig;
using tilt::estimation::ImuSample;
using tilt::estimation::Support;

int s_failures = 0;

void expectTrue(bool condition, const char* expression, const char* test_name) {
    if (!condition) {
        std::printf("FAIL [%s] %s\n", test_name, expression);
        ++s_failures;
    }
}

void expectNear(float actual, float expected, float tolerance,
                const char* expression, const char* test_name) {
    if (!(std::fabs(actual - expected) <= tolerance)) {
        std::printf("FAIL [%s] %s: expected %.5f, got %.5f (tol %.5f)\n",
                    test_name, expression, expected, actual, tolerance);
        ++s_failures;
    }
}

#define EXPECT_TRUE(t, e) expectTrue((e), #e, t)
#define EXPECT_NEAR(t, a, e, tol) expectNear((a), (e), (tol), #a, t)

constexpr float rad(float deg) { return deg * tilt::DEG2RAD; }

const float kStand[tilt::NUM_JOINTS] = {0.0f, rad(-20.0f), rad(40.0f),
                                        0.0f, rad(-20.0f), rad(40.0f)};

ImuSample level() {
    ImuSample imu;
    imu.valid = true;
    return imu;
}

void testComAgainstMujoco() {
    constexpr char kTest[] = "E1 CoM vs MuJoCo";
    float worst = 0.0f;
    for (const auto& s : kinematics_reference::SAMPLES) {
        float theta[tilt::NUM_JOINTS];
        for (int j = 0; j < 3; ++j) {
            theta[j] = s.left_theta[j];
            theta[3 + j] = s.right_theta[j];
        }
        const tilt::Vec3 c = tilt::estimation::center_of_mass(theta);
        const float dx = c.x - s.com_mm[0];
        const float dy = c.y - s.com_mm[1];
        const float dz = c.z - s.com_mm[2];
        const float e = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (e > worst) worst = e;
    }
    std::printf("[%s] worst CoM error %.4f mm, total mass %.5f kg\n", kTest,
                worst, tilt::estimation::total_mass_kg());
    EXPECT_TRUE(kTest, worst <= 0.05f);
    EXPECT_NEAR(kTest, tilt::estimation::total_mass_kg(), 1.23938f, 1e-4f);
}

void testStaticStand() {
    constexpr char kTest[] = "E2 static stand";
    DcmEstimator est;
    est.reset(kStand);
    for (int k = 0; k < 50; ++k) est.update(kStand, level(), Support::kDouble, 0.01f);
    const auto& s = est.state();
    std::printf("[%s] height %.2f mm, omega %.3f rad/s, com (%.2f, %.2f) mm, "
                "soles y %.2f / %.2f mm\n", kTest, s.height_mm, s.omega_rad_s,
                s.com_mm.x, s.com_mm.y, s.sole_mm[0].y, s.sole_mm[1].y);
    EXPECT_TRUE(kTest, s.valid);
    EXPECT_TRUE(kTest, s.height_mm > 200.0f && s.height_mm < 225.0f);
    EXPECT_NEAR(kTest, s.omega_rad_s, std::sqrt(9806.65f / s.height_mm), 1e-3f);
    EXPECT_NEAR(kTest, s.com_velocity_mm_s.y, 0.0f, 1e-3f);
    EXPECT_NEAR(kTest, s.dcm_mm[1], s.com_mm.y, 1e-3f);
    // Between the feet: neither inner edge has been passed.
    EXPECT_TRUE(kTest, s.dcm_past_inner_edge_mm[0] < -25.0f);
    EXPECT_TRUE(kTest, s.dcm_past_inner_edge_mm[1] < -25.0f);
}

void testRollMovesComTowardLowSide() {
    constexpr char kTest[] = "E3 roll geometry";
    DcmEstimator est;
    est.reset(kStand);
    ImuSample imu = level();
    est.update(kStand, imu, Support::kRight, 0.01f);
    const float y_level = est.state().com_mm.y;
    imu.roll_rad = rad(5.0f);  // Left side up: torso tilts to the right (-y).
    est.reset(kStand);
    est.update(kStand, imu, Support::kRight, 0.01f);
    const float y_rolled = est.state().com_mm.y;
    const float h = est.state().height_mm;
    std::printf("[%s] com y level %.2f -> rolled %.2f mm\n", kTest, y_level, y_rolled);
    EXPECT_TRUE(kTest, y_rolled < y_level);
    EXPECT_NEAR(kTest, y_level - y_rolled, h * std::sin(rad(5.0f)), 3.0f);
}

void testGyroVelocity() {
    constexpr char kTest[] = "E4 gyro -> velocity";
    DcmEstimator est;
    est.reset(kStand);
    ImuSample imu = level();
    imu.gyro_rad_s[0] = 1.0f;  // Rolling to the right at 1 rad/s.
    for (int k = 0; k < 30; ++k) est.update(kStand, imu, Support::kLeft, 0.01f);
    const auto& s = est.state();
    const float expected_vy = -s.com_mm.z * 1.0f;
    std::printf("[%s] vy %.1f mm/s (expected %.1f), dcm - com = %.2f mm\n",
                kTest, s.com_velocity_mm_s.y, expected_vy, s.dcm_mm[1] - s.com_mm.y);
    EXPECT_NEAR(kTest, s.com_velocity_mm_s.y, expected_vy, 2.0f);
    EXPECT_NEAR(kTest, s.dcm_mm[1] - s.com_mm.y, expected_vy / s.omega_rad_s, 0.5f);
}

void testSupportSwitchContinuity() {
    constexpr char kTest[] = "E5 support switch";
    DcmEstimator est;
    est.reset(kStand);
    for (int k = 0; k < 10; ++k) est.update(kStand, level(), Support::kLeft, 0.01f);
    est.update(kStand, level(), Support::kRight, 0.01f);
    const auto& s = est.state();
    EXPECT_NEAR(kTest, s.com_velocity_mm_s.y, 0.0f, 1e-3f);
    EXPECT_NEAR(kTest, s.com_velocity_mm_s.x, 0.0f, 1e-3f);
    // Reference moved to the right sole: the right sole is now the origin.
    EXPECT_NEAR(kTest, s.sole_mm[1].y, 0.0f, 1e-3f);
    EXPECT_TRUE(kTest, s.com_mm.y > 40.0f);
}

void testJointLag() {
    constexpr char kTest[] = "E6 joint lag model";
    EstimatorConfig config;
    config.joint_lag_s = 0.03f;
    DcmEstimator est(config);
    est.reset(kStand);
    float step[tilt::NUM_JOINTS];
    for (int j = 0; j < tilt::NUM_JOINTS; ++j) step[j] = kStand[j];
    step[tilt::R_KNEE_PITCH] += rad(15.0f);
    for (int k = 0; k < 3; ++k) est.update(step, level(), Support::kLeft, 0.01f);
    const float progress = (est.modelledTheta()[tilt::R_KNEE_PITCH] -
                            kStand[tilt::R_KNEE_PITCH]) / rad(15.0f);
    // Discrete first order: 1 - (0.03/0.04)^3 = 0.578.
    EXPECT_NEAR(kTest, progress, 1.0f - std::pow(0.75f, 3.0f), 1e-3f);
}

}  // namespace

int main() {
    testComAgainstMujoco();
    testStaticStand();
    testRollMovesComTowardLowSide();
    testGyroVelocity();
    testSupportSwitchContinuity();
    testJointLag();
    if (s_failures == 0) {
        std::printf("All estimation tests passed.\n");
        return 0;
    }
    std::printf("%d estimation test assertion(s) failed.\n", s_failures);
    return 1;
}
