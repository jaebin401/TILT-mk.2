#include <cmath>
#include <cstdint>
#include <cstdio>
#include <initializer_list>

#include "tilt_mpu6050.h"

namespace {

constexpr float kDegreesToRadians = 0.01745329251994329577f;
constexpr float kRadiansToDegrees = 57.2957795130823208768f;
constexpr float kAccelLsbPerG = 8192.0f;
constexpr float kGyroLsbPerDegreePerSecond = 65.5f;
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
        std::printf("FAIL [%s] %s: expected %.6f, got %.6f (tol %.6f)\n",
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

std::int16_t rawFromG(float acceleration_g) {
    return static_cast<std::int16_t>(std::lround(acceleration_g * kAccelLsbPerG));
}

std::int16_t rawFromDegreesPerSecond(float degrees_per_second) {
    return static_cast<std::int16_t>(
        std::lround(degrees_per_second * kGyroLsbPerDegreePerSecond));
}

tilt::ImuRaw gravityAtPitch(float pitch_rad, float gyro_y_degrees_per_second = 0.0f) {
    return {
        rawFromG(-std::sin(pitch_rad)),
        0,
        rawFromG(std::cos(pitch_rad)),
        0,
        rawFromDegreesPerSecond(gyro_y_degrees_per_second),
        0,
    };
}

void testStationaryConvergence() {
    constexpr char kTest[] = "F1 stationary convergence";
    tilt::ComplementaryFilter filter;
    const tilt::ImuRaw flat = gravityAtPitch(0.0f);
    for (int sample = 0; sample < 500; ++sample) {
        filter.update(flat, 0.01f);
    }
    const tilt::Attitude attitude = filter.attitude();
    EXPECT_NEAR(kTest, attitude.roll_rad, 0.0f, 0.5f * kDegreesToRadians);
    EXPECT_NEAR(kTest, attitude.pitch_rad, 0.0f, 0.5f * kDegreesToRadians);
}

void testTiltedStationaryState() {
    constexpr char kTest[] = "F2 tilted stationary state";
    constexpr float kTargetPitch = 10.0f * kDegreesToRadians;
    const tilt::ImuRaw tilted = gravityAtPitch(kTargetPitch);
    tilt::ComplementaryFilter filter;
    const tilt::Attitude first = filter.update(tilted, 0.01f);
    EXPECT_NEAR(kTest, first.pitch_rad, kTargetPitch, 0.5f * kDegreesToRadians);

    filter.reset();
    for (int sample = 0; sample < 500; ++sample) {
        filter.update(tilted, 0.01f);
    }
    EXPECT_NEAR(kTest, filter.attitude().pitch_rad, kTargetPitch,
                0.5f * kDegreesToRadians);
}

void testConstantAngularVelocity() {
    constexpr char kTest[] = "F3 constant angular velocity";
    constexpr float kRateDegreesPerSecond = 30.0f;
    tilt::ComplementaryFilter filter;
    filter.update(gravityAtPitch(0.0f), 0.01f);
    for (int sample = 1; sample <= 100; ++sample) {
        const float pitch = kRateDegreesPerSecond * kDegreesToRadians *
                            static_cast<float>(sample) * 0.01f;
        filter.update(gravityAtPitch(pitch, kRateDegreesPerSecond), 0.01f);
    }
    EXPECT_NEAR(kTest,
                filter.attitude().pitch_rad,
                30.0f * kDegreesToRadians,
                3.0f * kDegreesToRadians);
}

void testAccelerometerNoiseRejection() {
    constexpr char kTest[] = "F4 accelerometer noise rejection";
    tilt::ComplementaryFilter filter;
    const tilt::ImuRaw flat = gravityAtPitch(0.0f);
    for (int sample = 0; sample < 200; ++sample) {
        filter.update(flat, 0.01f);
    }

    std::uint32_t state = 0x12345678U;
    float maximum_angle = 0.0f;
    for (int sample = 0; sample < 500; ++sample) {
        state = state * 1664525U + 1013904223U;
        const float x_noise =
            (static_cast<float>((state >> 16U) & 0xFFFFU) / 65535.0f - 0.5f) *
            0.6f;
        state = state * 1664525U + 1013904223U;
        const float y_noise =
            (static_cast<float>((state >> 16U) & 0xFFFFU) / 65535.0f - 0.5f) *
            0.6f;
        const tilt::ImuRaw noisy = {
            rawFromG(x_noise), rawFromG(y_noise), rawFromG(1.0f), 0, 0, 0,
        };
        const tilt::Attitude attitude = filter.update(noisy, 0.01f);
        maximum_angle = std::fmax(maximum_angle, std::fabs(attitude.roll_rad));
        maximum_angle = std::fmax(maximum_angle, std::fabs(attitude.pitch_rad));
    }
    EXPECT_TRUE(kTest, maximum_angle < 5.0f * kDegreesToRadians);
}

void testInvalidDt() {
    constexpr char kTest[] = "F5 invalid dt";
    tilt::ComplementaryFilter filter;
    const tilt::ImuRaw tilted = gravityAtPitch(10.0f * kDegreesToRadians);
    filter.update(tilted, 0.01f);
    for (const float invalid_dt : {0.0f, -0.01f, 10.0f}) {
        const tilt::Attitude attitude = filter.update(tilted, invalid_dt);
        EXPECT_TRUE(kTest, std::isfinite(attitude.roll_rad));
        EXPECT_TRUE(kTest, std::isfinite(attitude.pitch_rad));
        EXPECT_NEAR(kTest, attitude.pitch_rad, 10.0f * kDegreesToRadians,
                    0.5f * kDegreesToRadians);
    }
}

}  // namespace

int main() {
    testStationaryConvergence();
    testTiltedStationaryState();
    testConstantAngularVelocity();
    testAccelerometerNoiseRejection();
    testInvalidDt();

    if (s_failures == 0) {
        std::printf("All complementary-filter tests passed.\n");
        return 0;
    }
    std::printf("%d complementary-filter assertion(s) failed.\n", s_failures);
    return 1;
}
