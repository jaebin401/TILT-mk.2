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
            gait.parameters().initial_settle_ms + gait.cycleDurationMs();
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
    tilt::gait::RockingGait gait(parameters);
    gait.start(tilt::gait::Side::kLeft);

    auto output = gait.update(parameters.transfer_ms);
    expect(output.position_rad[tilt::L_HIP_ROLL] < 0.0f,
           "left stance roll must shift the torso over the left foot");
    expect(std::fabs(output.position_rad[tilt::R_HIP_ROLL]) < 1.0e-6f,
           "right swing roll must remain neutral before lift");

    output = gait.update(parameters.lift_ms);
    expect(output.position_rad[tilt::R_HIP_ROLL] > 0.0f,
           "right swing foot must tuck toward the centerline");
    expect(std::fabs(output.position_rad[tilt::R_HIP_ROLL]) >
               std::fabs(output.position_rad[tilt::L_HIP_ROLL]),
           "swing roll magnitude must exceed stance roll magnitude");

    gait.update(parameters.hold_ms + parameters.lower_ms +
                parameters.touchdown_ms);
    output = gait.update(parameters.transfer_ms);
    expect(output.support_side == tilt::gait::Side::kRight,
           "support side must alternate");
    expect(output.position_rad[tilt::R_HIP_ROLL] > 0.0f,
           "right stance roll must shift the torso over the right foot");

    output = gait.update(parameters.lift_ms);
    expect(output.position_rad[tilt::L_HIP_ROLL] < 0.0f,
           "left swing foot must tuck toward the centerline");
}

void testContinuousSupportTransfer() {
    auto parameters = tilt::gait::parametersForPreset(
        tilt::gait::Preset::kVisualization);
    parameters.initial_settle_ms = 0;
    tilt::gait::RockingGait gait(parameters);
    gait.start(tilt::gait::Side::kLeft);

    const auto before_boundary = gait.update(gait.eventDurationMs() - 1);
    const auto after_boundary = gait.update(1);
    expect(after_boundary.support_side == tilt::gait::Side::kRight,
           "event boundary must immediately select the opposite support");
    expect(after_boundary.phase == tilt::gait::Phase::kTransfer,
           "event boundary must enter transfer without a DSP dwell");
    expect(std::fabs(after_boundary.position_rad[tilt::L_HIP_ROLL] -
                     before_boundary.position_rad[tilt::L_HIP_ROLL]) <
               1.0e-4f,
           "left hip roll must remain continuous at support change");
    expect(std::fabs(after_boundary.position_rad[tilt::R_HIP_ROLL] -
                     before_boundary.position_rad[tilt::R_HIP_ROLL]) <
               1.0e-4f,
           "right hip roll must remain continuous at support change");
    expect(gait.cycleDurationMs() == 1720,
           "visual preset must complete a left-right cycle in 1.72 seconds");
}

void testSwingFootPitchPreserved() {
    auto parameters = tilt::gait::parametersForPreset(
        tilt::gait::Preset::kVisualization);
    parameters.initial_settle_ms = 0;
    tilt::gait::RockingGait gait(parameters);
    gait.start(tilt::gait::Side::kLeft);
    const std::uint32_t lift_start = parameters.transfer_ms;
    const auto output = gait.update(lift_start + parameters.lift_ms);
    const float right_sum = output.position_rad[tilt::R_HIP_PITCH] +
                            output.position_rad[tilt::R_KNEE_PITCH];
    expect(std::fabs(right_sum - 20.0f * tilt::DEG2RAD) < 1.0e-5f,
           "swing hip+knee sum must preserve fixed-foot pitch");
    expect(output.position_rad[tilt::L_HIP_PITCH] == -20.0f * tilt::DEG2RAD,
           "stance pitch must remain at stand target");
}

}  // namespace

int main() {
    testSafeAndVisualLimits();
    testSupportSignsAndAlternation();
    testContinuousSupportTransfer();
    testSwingFootPitchPreserved();
    if (s_failures == 0) {
        std::printf("PASS: rocking gait host tests\n");
        return 0;
    }
    std::printf("FAIL: %d rocking gait checks\n", s_failures);
    return 1;
}
