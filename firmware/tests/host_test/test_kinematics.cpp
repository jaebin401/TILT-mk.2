#include <cmath>
#include <cstdio>
#include <initializer_list>

#include "tilt_kinematics.h"

namespace {

constexpr float kToleranceMm = 0.5f;
constexpr float kToleranceRad = 1.0e-4f;
int s_failures = 0;

void expectTrue(bool condition, const char* expression, const char* test_name) {
    if (!condition) {
        std::printf("FAIL [%s] %s\n", test_name, expression);
        ++s_failures;
    }
}

void expectNear(float actual,
                float expected,
                float tolerance,
                const char* expression,
                const char* test_name) {
    if (std::fabs(actual - expected) > tolerance) {
        std::printf("FAIL [%s] %s: expected %.7f, got %.7f (tol %.7f)\n",
                    test_name,
                    expression,
                    expected,
                    actual,
                    tolerance);
        ++s_failures;
    }
}

#define EXPECT_TRUE(test_name, expression) \
    expectTrue((expression), #expression, test_name)
#define EXPECT_NEAR(test_name, actual, expected, tolerance) \
    expectNear((actual), (expected), (tolerance), #actual, test_name)

constexpr float rad(float degrees) {
    return degrees * tilt::DEG2RAD;
}

void testZeroPose() {
    constexpr char kTest[] = "T1 zero pose";
    const float theta[] = {0.0f, rad(-20.0f), rad(+20.0f)};
    const tilt::Vec3 left = tilt::fk_foot(tilt::Leg::LEFT, theta);
    const tilt::Vec3 right = tilt::fk_foot(tilt::Leg::RIGHT, theta);

    EXPECT_NEAR(kTest, left.x, 0.0f, kToleranceMm);
    EXPECT_NEAR(kTest, left.y, +36.2f, kToleranceMm);
    EXPECT_NEAR(kTest, left.z, -103.97f, kToleranceMm);
    EXPECT_NEAR(kTest, right.x, 0.0f, kToleranceMm);
    EXPECT_NEAR(kTest, right.y, -36.2f, kToleranceMm);
    EXPECT_NEAR(kTest, right.z, -103.97f, kToleranceMm);
}

void testRoundTrip() {
    constexpr char kTest[] = "T2 FK/IK round trip";
    for (const tilt::Leg leg : {tilt::Leg::LEFT, tilt::Leg::RIGHT}) {
        for (int yaw_deg = -20; yaw_deg <= 20; yaw_deg += 5) {
            for (int hip_deg = -45; hip_deg <= 15; hip_deg += 5) {
                for (int knee_deg = 5; knee_deg <= 60; knee_deg += 5) {
                    const float theta[] = {
                        rad(static_cast<float>(yaw_deg)),
                        rad(static_cast<float>(hip_deg)),
                        rad(static_cast<float>(knee_deg)),
                    };
                    const tilt::Vec3 point = tilt::fk_foot(leg, theta);
                    const tilt::IkResult result =
                        tilt::ik_foot(leg, point, theta[0]);
                    EXPECT_TRUE(kTest, result.reachable);

                    const float local_y = point.y -
                        (leg == tilt::Leg::LEFT ? tilt::Y_HIP_MM
                                                 : -tilt::Y_HIP_MM);
                    const float horizontal_radius =
                        std::sqrt(point.x * point.x + local_y * local_y);
                    if (horizontal_radius >= tilt::YAW_SINGULARITY_EPS_MM) {
                        EXPECT_NEAR(kTest, result.theta[0], theta[0],
                                    kToleranceRad);
                    }
                    EXPECT_NEAR(kTest, result.theta[1], theta[1],
                                kToleranceRad);
                    EXPECT_NEAR(kTest, result.theta[2], theta[2],
                                kToleranceRad);
                }
            }
        }
    }
}

void testWorkspaceBoundary() {
    constexpr char kTest[] = "T3 workspace boundary";
    constexpr float kEffectiveLengthMm = 59.495315956686106f;
    const float maximum = tilt::THIGH_LENGTH_MM + kEffectiveLengthMm;
    const float minimum = std::fabs(tilt::THIGH_LENGTH_MM - kEffectiveLengthMm);

    const tilt::IkResult too_far = tilt::ik_foot(
        tilt::Leg::LEFT, {maximum + 10.0f, tilt::Y_HIP_MM, 0.0f}, 0.0f);
    const tilt::IkResult too_close = tilt::ik_foot(
        tilt::Leg::LEFT, {0.0f, tilt::Y_HIP_MM, 0.0f}, 0.0f);
    const tilt::IkResult inside_outer = tilt::ik_foot(
        tilt::Leg::LEFT, {maximum - 0.01f, tilt::Y_HIP_MM, 0.0f}, 0.0f);
    const tilt::IkResult inside_inner = tilt::ik_foot(
        tilt::Leg::LEFT, {minimum + 0.01f, tilt::Y_HIP_MM, 0.0f}, 0.0f);

    EXPECT_TRUE(kTest, !too_far.reachable);
    EXPECT_TRUE(kTest, !too_close.reachable);
    EXPECT_TRUE(kTest, inside_outer.reachable);
    EXPECT_TRUE(kTest, inside_inner.reachable);
    EXPECT_TRUE(kTest, std::isfinite(too_far.theta[0]));
    EXPECT_TRUE(kTest, std::isfinite(too_far.theta[1]));
    EXPECT_TRUE(kTest, std::isfinite(too_far.theta[2]));
}

void testMaximumExtensionHeight() {
    constexpr char kTest[] = "T4 maximum extension height";
    const float extension[] = {0.0f, 0.0f, 0.0f};
    const float zero[] = {0.0f, rad(-20.0f), rad(+20.0f)};
    const float extension_z = tilt::fk_foot(tilt::Leg::LEFT, extension).z;
    const float zero_z = tilt::fk_foot(tilt::Leg::LEFT, zero).z;
    std::printf("[%s] extension z=%.3f mm, zero-pose delta=%.3f mm\n",
                kTest,
                extension_z,
                zero_z - extension_z);
    EXPECT_TRUE(kTest, extension_z < zero_z);
}

void testYawSingularity() {
    constexpr char kTest[] = "T5 hip yaw singularity";
    constexpr float kHints[] = {-0.3f, 0.0f, +0.3f};
    for (const float hint : kHints) {
        const tilt::IkResult left = tilt::ik_foot(
            tilt::Leg::LEFT, {0.0f, +36.2f, -103.97f}, hint);
        const tilt::IkResult right = tilt::ik_foot(
            tilt::Leg::RIGHT, {0.0f, -36.2f, -103.97f}, hint);
        EXPECT_NEAR(kTest, left.theta[0], hint, kToleranceRad);
        EXPECT_NEAR(kTest, right.theta[0], hint, kToleranceRad);
    }
}

void testLeftRightSymmetry() {
    constexpr char kTest[] = "T6 left/right symmetry";
    // At zero pose the foot is directly below Hip Yaw (u = 0), so the same
    // joint angles are a true left/right mirror in the torso coordinate frame.
    const float theta[] = {rad(15.0f), rad(-20.0f), rad(+20.0f)};
    const tilt::Vec3 left = tilt::fk_foot(tilt::Leg::LEFT, theta);
    const tilt::Vec3 right = tilt::fk_foot(tilt::Leg::RIGHT, theta);
    EXPECT_NEAR(kTest, left.x, right.x, kToleranceMm);
    EXPECT_NEAR(kTest, left.y, -right.y, kToleranceMm);
    EXPECT_NEAR(kTest, left.z, right.z, kToleranceMm);
}

}  // namespace

int main() {
    testZeroPose();
    testRoundTrip();
    testWorkspaceBoundary();
    testMaximumExtensionHeight();
    testYawSingularity();
    testLeftRightSymmetry();

    if (s_failures == 0) {
        std::printf("All kinematics tests passed.\n");
        return 0;
    }
    std::printf("%d kinematics test assertion(s) failed.\n", s_failures);
    return 1;
}
