#include <array>
#include <cmath>
#include <cstdio>

#include "BodyPoseController.h"
#include "tilt_kinematics.h"

namespace {

using tilt::wheel_base::BodyPoseController;
using tilt::wheel_base::JointArray;

int s_failures = 0;

void expect(bool condition, const char* message) {
    if (!condition) {
        std::printf("FAIL: %s\n", message);
        ++s_failures;
    }
}

JointArray standPose() {
    JointArray pose{};
    for (int joint = 0; joint < tilt::NUM_JOINTS; ++joint) {
        pose[joint] = tilt::ZERO_POSE_RAD[joint];
    }
    return pose;
}

tilt::Vec3 foot(const JointArray& q, int leg) {
    float joints[tilt::NUM_LEG_JOINTS]{};
    for (int joint = 0; joint < tilt::NUM_LEG_JOINTS; ++joint) {
        joints[joint] = q[tilt::jointIndex(leg, joint)];
    }
    return tilt::fk_foot(leg == 0 ? tilt::Leg::LEFT : tilt::Leg::RIGHT,
                         joints);
}

bool runUntilSettled(BodyPoseController& controller, JointArray& command) {
    for (int cycle = 0; cycle < 500; ++cycle) {
        if (!controller.update(0.01f, command)) {
            const auto& c = controller.current();
            const auto& t = controller.target();
            std::printf("solve rejected at cycle %d current=(%.3f, %.3f, %.3fdeg) "
                        "target=(%.3f, %.3f, %.3fdeg)\n",
                        cycle, c.x_mm, c.z_mm,
                        c.hip_pitch_offset_rad / tilt::DEG2RAD,
                        t.x_mm, t.z_mm,
                        t.hip_pitch_offset_rad / tilt::DEG2RAD);
            return false;
        }
    }
    return true;
}

void testStandTranslationAndHipControl() {
    BodyPoseController controller;
    const JointArray stand = standPose();
    const tilt::Vec3 reference_left = foot(stand, 0);
    const tilt::Vec3 reference_right = foot(stand, 1);
    controller.captureReference(stand);

    JointArray command{};
    expect(controller.update(0.01f, command), "STAND reference solves");

    controller.nudgePosition(2.0f, 0.0f);
    expect(runUntilSettled(controller, command), "2 mm x translation solves");
    const tilt::Vec3 moved_left = foot(command, 0);
    const tilt::Vec3 moved_right = foot(command, 1);
    expect(std::fabs(moved_left.x - (reference_left.x - 2.0f)) < 0.01f,
           "left foot x follows body translation");
    expect(std::fabs(moved_right.x - (reference_right.x - 2.0f)) < 0.01f,
           "right foot x follows body translation");
    expect(std::fabs(moved_left.z - reference_left.z) < 0.01f,
           "x translation preserves left foot z");
    expect(std::fabs(command[tilt::L_HIP_PITCH] -
                     stand[tilt::L_HIP_PITCH]) < 1.0e-5f,
           "translation holds left hip pitch");
    expect(std::fabs(command[tilt::R_HIP_PITCH] -
                     stand[tilt::R_HIP_PITCH]) < 1.0e-5f,
           "translation holds right hip pitch");

    controller.nudgeHipPitch(1.0f * tilt::DEG2RAD);
    expect(runUntilSettled(controller, command), "hip pitch adjustment solves");
    expect(std::fabs(command[tilt::L_HIP_PITCH] -
                     (-19.0f * tilt::DEG2RAD)) < 1.0e-4f,
           "left hip reaches requested pitch");
    expect(std::fabs(command[tilt::R_HIP_PITCH] -
                     (-19.0f * tilt::DEG2RAD)) < 1.0e-4f,
           "right hip reaches requested pitch");
    const tilt::Vec3 hip_left = foot(command, 0);
    expect(std::fabs(hip_left.x - moved_left.x) < 0.01f &&
           std::fabs(hip_left.z - moved_left.z) < 0.01f,
           "hip adjustment preserves sole x/z");

    controller.nudgePosition(0.0f, -2.0f);
    expect(runUntilSettled(controller, command), "2 mm downward translation solves");
    const tilt::Vec3 lowered_left = foot(command, 0);
    expect(std::fabs(lowered_left.z - (reference_left.z + 2.0f)) < 0.01f,
           "downward body translation moves relative foot target up");
}

void testHomeReference() {
    BodyPoseController controller;
    JointArray home{};
    controller.captureReference(home);
    JointArray command{};
    const bool solved = controller.update(0.01f, command);
    expect(solved, "HOME reference solves");
    for (int joint = 0; joint < tilt::NUM_JOINTS; ++joint) {
        expect(std::fabs(command[joint]) < 1.0e-4f,
               "HOME reference stays at zero");
    }
}

}  // namespace

int main() {
    testStandTranslationAndHipControl();
    testHomeReference();
    if (s_failures == 0) {
        std::printf("wheel body pose tests passed\n");
        return 0;
    }
    std::printf("%d wheel body pose failure(s)\n", s_failures);
    return 1;
}
