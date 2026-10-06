#include <cmath>
#include <cstdio>
#include <cstring>

#include "JointMap.h"
#include "PosePlayer.h"
#include "PoseTable.h"

namespace {

constexpr float kFloatTolerance = 1.0e-5f;
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

float rad(float degrees) {
    return degrees * tilt::DEG2RAD;
}

void testConversionRoundTrip() {
    constexpr char kTest[] = "T1 conversion round trip";
    const float tolerance = tilt::RAD_PER_TICK * 0.5001f;

    for (int joint = 0; joint < tilt::NUM_JOINTS; ++joint) {
        const tilt::JointLimit& limit = tilt::JOINT_LIMIT[joint / 3][joint % 3];
        for (float angle = limit.minimum_rad;
             angle <= limit.maximum_rad + 1.0e-6f;
             angle += tilt::DEG2RAD) {
            const float bounded = angle > limit.maximum_rad
                                      ? limit.maximum_rad
                                      : angle;
            std::uint16_t tick = 0;
            EXPECT_TRUE(kTest, tilt::pose::radToTick(joint, bounded, tick));
            EXPECT_NEAR(kTest,
                        tilt::pose::tickToRad(joint, tick),
                        bounded,
                        tolerance);
        }

        std::uint16_t maximum_tick = 0;
        EXPECT_TRUE(kTest,
                    tilt::pose::radToTick(joint,
                                          limit.maximum_rad,
                                          maximum_tick));
        EXPECT_NEAR(kTest,
                    tilt::pose::tickToRad(joint, maximum_tick),
                    limit.maximum_rad,
                    tolerance);
    }
}

void testZeroAndSign() {
    constexpr char kTest[] = "T2 zero and sign";
    for (int joint = 0; joint < tilt::NUM_JOINTS; ++joint) {
        std::uint16_t zero_tick = 0;
        EXPECT_TRUE(kTest, tilt::pose::radToTick(joint, 0.0f, zero_tick));
        EXPECT_TRUE(kTest, zero_tick == tilt::ZERO_TICK[joint]);

        std::uint16_t positive_tick = 0;
        EXPECT_TRUE(kTest,
                    tilt::pose::radToTick(joint, rad(10.0f), positive_tick));
        const int delta = static_cast<int>(positive_tick) -
                          static_cast<int>(tilt::ZERO_TICK[joint]);
        EXPECT_TRUE(kTest,
                    (delta > 0 ? +1 : -1) == tilt::JOINT_SIGN[joint]);
    }
}

void expectWithinLimits(const tilt::pose::JointArray& pose,
                        const char* expression,
                        const char* test_name) {
    int bad_joint = -1;
    if (!tilt::pose::withinLimits(pose, bad_joint)) {
        std::printf("FAIL [%s] %s: joint %d is outside its limit\n",
                    test_name,
                    expression,
                    bad_joint);
        ++s_failures;
    }
}

void testPoseLimitsAndFeet() {
    constexpr char kTest[] = "T3 pose limits and feet";
    expectWithinLimits(tilt::pose::homePose(), "homePose", kTest);
    expectWithinLimits(tilt::pose::standPose(), "standPose", kTest);
    expectWithinLimits(tilt::pose::shiftPose(tilt::pose::Side::LEFT),
                       "shiftPose LEFT", kTest);
    expectWithinLimits(tilt::pose::shiftPose(tilt::pose::Side::RIGHT),
                       "shiftPose RIGHT", kTest);
    expectWithinLimits(tilt::pose::liftPose(tilt::pose::Side::LEFT),
                       "liftPose LEFT support", kTest);
    expectWithinLimits(tilt::pose::liftPose(tilt::pose::Side::RIGHT),
                       "liftPose RIGHT support", kTest);

    const auto stand = tilt::pose::standPose();
    EXPECT_TRUE(kTest,
                std::fabs(tilt::pose::footPitchRad(stand, 0)) <= rad(0.5f));
    EXPECT_TRUE(kTest,
                std::fabs(tilt::pose::footPitchRad(stand, 1)) <= rad(0.5f));
}

void testPoseSymmetry() {
    constexpr char kTest[] = "T4 pose symmetry";
    const auto stand = tilt::pose::standPose();
    for (int joint = 0; joint < 3; ++joint) {
        EXPECT_NEAR(kTest,
                    stand[joint],
                    stand[joint + 3],
                    kFloatTolerance);
    }

    const auto lift_left = tilt::pose::liftPose(tilt::pose::Side::RIGHT);
    const auto lift_right = tilt::pose::liftPose(tilt::pose::Side::LEFT);
    EXPECT_NEAR(kTest,
                lift_left[tilt::L_HIP_ROLL],
                -lift_right[tilt::L_HIP_ROLL],
                kFloatTolerance);
    EXPECT_NEAR(kTest,
                lift_left[tilt::R_HIP_ROLL],
                -lift_right[tilt::R_HIP_ROLL],
                kFloatTolerance);
    EXPECT_NEAR(kTest,
                lift_left[tilt::L_HIP_PITCH] - stand[tilt::L_HIP_PITCH],
                lift_right[tilt::R_HIP_PITCH] - stand[tilt::R_HIP_PITCH],
                kFloatTolerance);
    EXPECT_NEAR(kTest,
                lift_left[tilt::L_KNEE_PITCH] - stand[tilt::L_KNEE_PITCH],
                lift_right[tilt::R_KNEE_PITCH] - stand[tilt::R_KNEE_PITCH],
                kFloatTolerance);

    if (std::fabs(tilt::pose::kLiftHipDeltaDeg +
                  tilt::pose::kLiftKneeDeltaDeg) <= kFloatTolerance) {
        EXPECT_NEAR(kTest,
                    tilt::pose::footPitchRad(lift_left, 0),
                    tilt::pose::footPitchRad(
                        tilt::pose::shiftPose(tilt::pose::Side::RIGHT), 0),
                    kFloatTolerance);
        EXPECT_NEAR(kTest,
                    tilt::pose::footPitchRad(lift_right, 1),
                    tilt::pose::footPitchRad(
                        tilt::pose::shiftPose(tilt::pose::Side::LEFT), 1),
                    kFloatTolerance);
    }
}

void testInterpolation() {
    constexpr char kTest[] = "T5 interpolation";
    tilt::pose::PosePlayer player;
    tilt::pose::JointArray start{};
    tilt::pose::JointArray target{};
    target.fill(1.0f);
    player.reset(start);
    EXPECT_TRUE(kTest, player.enqueue({target, 100, "TARGET"}));

    float previous = 0.0f;
    float first_delta = 0.0f;
    float last_delta = 0.0f;
    for (int step = 1; step <= 10; ++step) {
        const auto value = player.step(10);
        EXPECT_TRUE(kTest, value[0] >= previous);
        const float delta = value[0] - previous;
        if (step == 1) first_delta = delta;
        if (step == 10) last_delta = delta;
        previous = value[0];
    }
    EXPECT_NEAR(kTest, previous, 1.0f, kFloatTolerance);
    EXPECT_TRUE(kTest, player.idle());
    EXPECT_NEAR(kTest, first_delta, last_delta, kFloatTolerance);
    EXPECT_TRUE(kTest, first_delta < 0.1f);

    const auto held = player.step(1000);
    EXPECT_NEAR(kTest, held[0], 1.0f, kFloatTolerance);

    player.reset(start);
    EXPECT_TRUE(kTest, player.enqueue({target, 100, "HALF"}));
    const auto halfway = player.step(50);
    EXPECT_NEAR(kTest, halfway[0], 0.5f, kFloatTolerance);
    const auto finished = player.step(50);
    EXPECT_NEAR(kTest, finished[0], 1.0f, kFloatTolerance);
}

void expectSegmentName(const tilt::pose::Sequence& sequence,
                       std::size_t index,
                       const char* expected,
                       const char* test_name) {
    EXPECT_TRUE(test_name, index < sequence.count);
    if (index < sequence.count) {
        EXPECT_TRUE(test_name,
                    std::strcmp(sequence.segments[index].name, expected) == 0);
    }
}

void testSequences() {
    constexpr char kTest[] = "T6 sequences";
    tilt::pose::Sequence sequence;

    EXPECT_TRUE(kTest,
                tilt::pose::makeSequence(tilt::pose::Posture::STAND,
                                         tilt::pose::PoseCommand::LIFT_L,
                                         sequence));
    EXPECT_TRUE(kTest, sequence.count == 2);
    expectSegmentName(sequence, 0, "SHIFT_R", kTest);
    expectSegmentName(sequence, 1, "LIFT_L", kTest);

    EXPECT_TRUE(kTest,
                tilt::pose::makeSequence(tilt::pose::Posture::LIFT_L,
                                         tilt::pose::PoseCommand::STAND,
                                         sequence));
    EXPECT_TRUE(kTest, sequence.count == 2);
    expectSegmentName(sequence, 0, "SHIFT_R", kTest);
    expectSegmentName(sequence, 1, "STAND", kTest);

    EXPECT_TRUE(kTest,
                tilt::pose::makeSequence(tilt::pose::Posture::LIFT_L,
                                         tilt::pose::PoseCommand::LIFT_R,
                                         sequence));
    EXPECT_TRUE(kTest, sequence.count == 4);
    expectSegmentName(sequence, 0, "SHIFT_R", kTest);
    expectSegmentName(sequence, 1, "STAND", kTest);
    expectSegmentName(sequence, 2, "SHIFT_L", kTest);
    expectSegmentName(sequence, 3, "LIFT_R", kTest);

    EXPECT_TRUE(kTest,
                !tilt::pose::makeSequence(tilt::pose::Posture::HOME,
                                          tilt::pose::PoseCommand::LIFT_L,
                                          sequence));
}

}  // namespace

int main() {
    testConversionRoundTrip();
    testZeroAndSign();
    testPoseLimitsAndFeet();
    testPoseSymmetry();
    testInterpolation();
    testSequences();

    if (s_failures == 0) {
        std::printf("All pose tests passed.\n");
        return 0;
    }
    std::printf("%d pose test assertion(s) failed.\n", s_failures);
    return 1;
}
