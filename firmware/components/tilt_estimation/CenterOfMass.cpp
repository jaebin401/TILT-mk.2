#include "tilt/estimation/CenterOfMass.h"

#include "tilt/kinematics/RobotModel.h"

namespace tilt::estimation {
namespace {

struct Accumulator {
    float mass = 0.0f;
    float mx = 0.0f;
    float my = 0.0f;
    float mz = 0.0f;

    void add(const Pose& frame, const model::LinkInertial& link) {
        const Vec3 p = transform_point(
            frame, {link.com_mm[0], link.com_mm[1], link.com_mm[2]});
        mass += link.mass_kg;
        mx += link.mass_kg * p.x;
        my += link.mass_kg * p.y;
        mz += link.mass_kg * p.z;
    }
};

}  // namespace

Vec3 center_of_mass(const float leg_theta[NUM_JOINTS],
                    const UpperBodyPose& upper) {
    Accumulator acc;
    const Pose base = base_pose();
    acc.add(base, model::TORSO);

    for (int leg = 0; leg < 2; ++leg) {
        const LegFrames frames =
            fk_leg(static_cast<Leg>(leg), &leg_theta[leg * 3]);
        acc.add(frames.hip, model::LEG_LINK[leg][0]);
        acc.add(frames.thigh, model::LEG_LINK[leg][1]);
        acc.add(frames.calf, model::LEG_LINK[leg][2]);
    }

    acc.add(joint_child_pose(base, model::HEAD_PITCH, upper.head_pitch_rad),
            model::HEAD);
    for (int side = 0; side < 2; ++side) {
        const Pose shoulder = joint_child_pose(
            base, model::ARM_JOINT[side][0], upper.arm_pitch_rad[side]);
        const Pose arm = joint_child_pose(
            shoulder, model::ARM_JOINT[side][1], upper.arm_roll_rad[side]);
        acc.add(shoulder, model::ARM_LINK[side][0]);
        acc.add(arm, model::ARM_LINK[side][1]);
    }

    return {acc.mx / acc.mass, acc.my / acc.mass, acc.mz / acc.mass};
}

float total_mass_kg() {
    return model::TOTAL_MASS_KG;
}

}  // namespace tilt::estimation
