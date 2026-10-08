#include <cmath>
#include <cstdio>

#include "tilt/gait/RockingGait.h"

namespace {

int s_failures = 0;

void expect(bool condition, const char* message) {
    if (!condition) {
        std::printf("FAIL: %s\n", message);
        ++s_failures;
    }
}

bool withinLimits(const tilt::gait::JointArray& q) {
    for (int joint = 0; joint < tilt::NUM_JOINTS; ++joint) {
        const auto& limit = tilt::JOINT_LIMIT[joint / 3][joint % 3];
        if (q[joint] < limit.minimum_rad - 1.0e-6f ||
            q[joint] > limit.maximum_rad + 1.0e-6f) {
            return false;
        }
    }
    return true;
}

void testSafeAndVisualLimits() {
    for (const auto preset : {tilt::gait::Preset::kConservative,
                              tilt::gait::Preset::kVisualization}) {
        tilt::gait::RockingGait gait(
            tilt::gait::parametersForPreset(preset));
        gait.start();
        const std::uint32_t duration =
            gait.parameters().initial_settle_ms +
            (gait.parameters().build_rock_halves +
             gait.parameters().lift_ramp_halves + 2) *
                gait.eventDurationMs();
        for (std::uint32_t elapsed = 0; elapsed <= duration; elapsed += 10) {
            expect(withinLimits(gait.update(10).position_rad),
                   "preset command outside joint limits");
        }
    }
}

void testSupportSignsAndAlternation() {
    auto parameters = tilt::gait::parametersForPreset(
        tilt::gait::Preset::kVisualization);
    parameters.initial_settle_ms = 0;
    parameters.build_rock_halves = 0;
    parameters.lift_ramp_halves = 0;
    tilt::gait::RockingGait gait(parameters);
    gait.start(tilt::gait::Side::kLeft);

    const std::uint32_t early = parameters.half_cycle_ms / 20;
    const std::uint32_t peak = parameters.half_cycle_ms / 2;
    auto output = gait.update(early);
    expect(output.position_rad[tilt::L_HIP_ROLL] < 0.0f,
           "left stance roll must shift the torso over the left foot");
    expect(output.position_rad[tilt::R_HIP_ROLL] > 0.0f,
           "right swing roll must start immediately without a DSP window");

    output = gait.update(peak - early);
    expect(output.position_rad[tilt::R_HIP_ROLL] > 0.0f,
           "right swing foot must tuck toward the centerline");
    expect(std::fabs(output.position_rad[tilt::R_HIP_ROLL]) >
               std::fabs(output.position_rad[tilt::L_HIP_ROLL]),
           "swing roll magnitude must exceed stance roll magnitude");

    gait.update(parameters.half_cycle_ms - peak);
    output = gait.update(early);
    expect(output.support_side == tilt::gait::Side::kRight,
           "support side must alternate");
    expect(output.position_rad[tilt::R_HIP_ROLL] > 0.0f,
           "right stance roll must shift the torso over the right foot");

    output = gait.update(peak - early);
    expect(output.position_rad[tilt::L_HIP_ROLL] < 0.0f,
           "left swing foot must tuck toward the centerline");
}

void testContinuousSupportTransfer() {
    auto parameters = tilt::gait::parametersForPreset(
        tilt::gait::Preset::kVisualization);
    parameters.initial_settle_ms = 0;
    parameters.build_rock_halves = 0;
    parameters.lift_ramp_halves = 0;
    tilt::gait::RockingGait gait(parameters);
    gait.start(tilt::gait::Side::kLeft);

    const auto before_boundary = gait.update(gait.eventDurationMs() - 1);
    const auto after_boundary = gait.update(1);
    expect(after_boundary.support_side == tilt::gait::Side::kRight,
           "event boundary must immediately select the opposite support");
    expect(after_boundary.phase == tilt::gait::Phase::kLift,
           "event boundary must enter the next lift without a transfer phase");
    expect(std::fabs(after_boundary.position_rad[tilt::L_HIP_ROLL] -
                     before_boundary.position_rad[tilt::L_HIP_ROLL]) <
               3.0e-3f,
           "left hip roll must remain continuous at support change");
    expect(std::fabs(after_boundary.position_rad[tilt::R_HIP_ROLL] -
                     before_boundary.position_rad[tilt::R_HIP_ROLL]) <
               3.0e-3f,
           "right hip roll must remain continuous at support change");
    const auto moving_after_boundary = gait.update(10);
    expect(moving_after_boundary.position_rad[tilt::L_HIP_PITCH] <
               -20.0f * tilt::DEG2RAD,
           "opposite swing leg must lift on the first control tick");
    expect(gait.cycleDurationMs() == 920,
           "visual preset must complete a left-right cycle in 0.92 seconds");
}

void testSwingFootPitchPreserved() {
    auto parameters = tilt::gait::parametersForPreset(
        tilt::gait::Preset::kVisualization);
    parameters.initial_settle_ms = 0;
    parameters.build_rock_halves = 0;
    parameters.lift_ramp_halves = 0;
    tilt::gait::RockingGait gait(parameters);
    gait.start(tilt::gait::Side::kLeft);
    const auto output = gait.update(parameters.half_cycle_ms / 2);
    const float right_sum = output.position_rad[tilt::R_HIP_PITCH] +
                            output.position_rad[tilt::R_KNEE_PITCH];
    expect(std::fabs(right_sum - 20.0f * tilt::DEG2RAD) < 1.0e-5f,
           "swing hip+knee sum must preserve fixed-foot pitch");
    expect(output.position_rad[tilt::L_HIP_PITCH] == -20.0f * tilt::DEG2RAD,
           "stance pitch must remain at stand target");
}

void testCustomLiftHeight() {
    auto parameters = tilt::gait::parametersForPreset(
        tilt::gait::Preset::kVisualization);
    parameters.initial_settle_ms = 0;
    parameters.build_rock_halves = 0;
    parameters.lift_ramp_halves = 0;
    parameters.lift_hip_delta_rad = -20.0f * tilt::DEG2RAD;
    parameters.lift_knee_delta_rad = +20.0f * tilt::DEG2RAD;
    tilt::gait::RockingGait gait(parameters);
    gait.start(tilt::gait::Side::kLeft);

    const auto output = gait.update(parameters.half_cycle_ms / 2);
    expect(std::fabs(output.position_rad[tilt::R_HIP_PITCH] -
                     -40.0f * tilt::DEG2RAD) < 1.0e-5f,
           "custom lift must reach the configured hip-pitch height");
    expect(std::fabs(output.position_rad[tilt::R_KNEE_PITCH] -
                     +60.0f * tilt::DEG2RAD) < 1.0e-5f,
           "custom lift must reach the configured knee-pitch height");
    const float right_sum = output.position_rad[tilt::R_HIP_PITCH] +
                            output.position_rad[tilt::R_KNEE_PITCH];
    expect(std::fabs(right_sum - 20.0f * tilt::DEG2RAD) < 1.0e-5f,
           "custom lift must preserve the fixed-foot pitch");
    expect(withinLimits(output.position_rad),
           "maximum custom lift must remain inside joint limits");
}

void testCustomFastPeriod() {
    auto parameters = tilt::gait::parametersForPreset(
        tilt::gait::Preset::kVisualization);
    parameters.initial_settle_ms = 0;
    parameters.build_rock_halves = 0;
    parameters.lift_ramp_halves = 0;
    parameters.half_cycle_ms = 400;
    tilt::gait::RockingGait gait(parameters);
    gait.start(tilt::gait::Side::kLeft);

    expect(gait.eventDurationMs() == 400,
           "custom fast half-cycle must be applied");
    expect(gait.cycleDurationMs() == 800,
           "custom fast full cycle must be twice the half-cycle");
    gait.update(400);
    expect(gait.output().support_side == tilt::gait::Side::kRight,
           "custom fast period must switch support at its boundary");
}

void testRockBuildBeforeLift() {
    auto parameters = tilt::gait::parametersForPreset(
        tilt::gait::Preset::kVisualization);
    parameters.initial_settle_ms = 0;
    tilt::gait::RockingGait gait(parameters);
    gait.start(tilt::gait::Side::kLeft);

    const auto output = gait.update(parameters.half_cycle_ms / 2);
    expect(output.position_rad[tilt::L_HIP_ROLL] < 0.0f,
           "startup must rock toward the support side");
    expect(std::fabs(output.position_rad[tilt::R_HIP_ROLL]) < 1.0e-6f,
           "startup must keep the swing hip down while building momentum");
    expect(output.position_rad[tilt::R_HIP_PITCH] ==
               -20.0f * tilt::DEG2RAD,
           "startup must not lift a foot before rock build completes");
}

void testPitchOnlyMode() {
    auto parameters = tilt::gait::parametersForPreset(
        tilt::gait::Preset::kVisualization);
    parameters.initial_settle_ms = 0;
    parameters.hip_roll_mode = tilt::gait::HipRollMode::kDisabled;
    parameters.lift_ramp_halves = 0;
    tilt::gait::RockingGait gait(parameters);
    gait.start(tilt::gait::Side::kLeft);

    const auto output = gait.update(parameters.half_cycle_ms / 2);
    expect(std::fabs(output.position_rad[tilt::L_HIP_ROLL]) < 1.0e-6f &&
               std::fabs(output.position_rad[tilt::R_HIP_ROLL]) < 1.0e-6f,
           "pitch-only mode must keep both hip-roll targets at zero");
    expect(output.position_rad[tilt::R_HIP_PITCH] <
               -20.0f * tilt::DEG2RAD,
           "pitch-only mode must still lift the swing leg");
    expect(output.position_rad[tilt::R_KNEE_PITCH] >
               +40.0f * tilt::DEG2RAD,
           "pitch-only mode must still flex the swing knee");
}

void testCustomHipRollPoseMirrors() {
    auto parameters = tilt::gait::parametersForPreset(
        tilt::gait::Preset::kVisualization);
    parameters.initial_settle_ms = 0;
    parameters.build_rock_halves = 0;
    parameters.lift_ramp_halves = 0;
    parameters.hip_roll_mode = tilt::gait::HipRollMode::kCustom;
    parameters.custom_left_support_left_roll_rad = -3.0f * tilt::DEG2RAD;
    parameters.custom_left_support_right_roll_rad = +7.0f * tilt::DEG2RAD;
    tilt::gait::RockingGait gait(parameters);
    gait.start(tilt::gait::Side::kLeft);

    auto output = gait.update(parameters.half_cycle_ms / 2);
    expect(std::fabs(output.position_rad[tilt::L_HIP_ROLL] -
                     parameters.custom_left_support_left_roll_rad) < 1.0e-6f,
           "custom left-support LHR must be reproduced at the peak");
    expect(std::fabs(output.position_rad[tilt::R_HIP_ROLL] -
                     parameters.custom_left_support_right_roll_rad) < 1.0e-6f,
           "custom left-support RHR must be reproduced at the peak");

    output = gait.update(parameters.half_cycle_ms);
    expect(output.support_side == tilt::gait::Side::kRight,
           "custom pose must alternate to right support");
    expect(std::fabs(output.position_rad[tilt::L_HIP_ROLL] +
                     parameters.custom_left_support_right_roll_rad) < 1.0e-6f,
           "right-support LHR must mirror saved RHR");
    expect(std::fabs(output.position_rad[tilt::R_HIP_ROLL] +
                     parameters.custom_left_support_left_roll_rad) < 1.0e-6f,
           "right-support RHR must mirror saved LHR");
}

}  // namespace

int main() {
    testSafeAndVisualLimits();
    testSupportSignsAndAlternation();
    testContinuousSupportTransfer();
    testSwingFootPitchPreserved();
    testCustomLiftHeight();
    testCustomFastPeriod();
    testRockBuildBeforeLift();
    testPitchOnlyMode();
    testCustomHipRollPoseMirrors();
    if (s_failures == 0) {
        std::printf("PASS: rocking gait host tests\n");
        return 0;
    }
    std::printf("FAIL: %d rocking gait checks\n", s_failures);
    return 1;
}
