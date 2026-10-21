#include "BodyPoseController.h"

#include <algorithm>
#include <cmath>

namespace tilt::wheel_base {
namespace {

constexpr float kMaximumXmm = 15.0f;
constexpr float kMaximumZmm = 15.0f;
constexpr float kMaximumHipOffsetRad = 15.0f * DEG2RAD;
constexpr float kLinearSpeedMmPerSecond = 15.0f;
constexpr float kHipSpeedRadPerSecond = 10.0f * DEG2RAD;

float moveToward(float current, float target, float maximum_delta) {
    if (current < target) return std::min(current + maximum_delta, target);
    if (current > target) return std::max(current - maximum_delta, target);
    return current;
}

}  // namespace

void BodyPoseController::captureReference(const JointArray& pose) {
    last_command_ = pose;
    for (int leg = 0; leg < 2; ++leg) {
        float joints[NUM_LEG_JOINTS]{};
        for (int joint = 0; joint < NUM_LEG_JOINTS; ++joint) {
            joints[joint] = pose[jointIndex(leg, joint)];
        }
        reference_foot_[leg] =
            fk_foot(leg == 0 ? Leg::LEFT : Leg::RIGHT, joints);
        reference_hip_roll_[leg] = joints[0];
        reference_hip_pitch_[leg] = joints[1];
    }
    current_ = {};
    target_ = {};
    initialized_ = true;
}

void BodyPoseController::nudgePosition(float x_mm, float z_mm) {
    if (!initialized_) return;
    target_.x_mm = std::clamp(target_.x_mm + x_mm,
                              -kMaximumXmm, kMaximumXmm);
    target_.z_mm = std::clamp(target_.z_mm + z_mm,
                              -kMaximumZmm, kMaximumZmm);
}

void BodyPoseController::nudgeHipPitch(float radians) {
    if (!initialized_) return;
    target_.hip_pitch_offset_rad = std::clamp(
        target_.hip_pitch_offset_rad + radians,
        -kMaximumHipOffsetRad, kMaximumHipOffsetRad);
}

void BodyPoseController::resetTarget() {
    if (!initialized_) return;
    target_ = {};
}

bool BodyPoseController::solve(const BodyTarget& body,
                               JointArray& command) const {
    command = last_command_;
    for (int leg = 0; leg < 2; ++leg) {
        Vec3 target = reference_foot_[leg];
        target.x -= body.x_mm;
        target.z -= body.z_mm;

        const int base = leg * NUM_LEG_JOINTS;
        const IkResult result = ik_foot_fixed_hip_pitch(
            leg == 0 ? Leg::LEFT : Leg::RIGHT,
            target,
            reference_hip_roll_[leg],
            reference_hip_pitch_[leg] + body.hip_pitch_offset_rad,
            last_command_[base + 2],
            last_command_[base + 3]);
        if (!result.reachable) return false;

        for (int joint = 0; joint < NUM_LEG_JOINTS; ++joint) {
            const float value = result.theta[joint];
            const JointLimit& limit = JOINT_LIMIT[leg][joint];
            if (!std::isfinite(value) || value < limit.minimum_rad ||
                value > limit.maximum_rad) {
                return false;
            }
            command[base + joint] = value;
        }
    }
    return true;
}

bool BodyPoseController::update(float dt_seconds, JointArray& command) {
    if (!initialized_ || !std::isfinite(dt_seconds) || dt_seconds <= 0.0f) {
        return false;
    }

    const BodyTarget previous = current_;
    current_.x_mm = moveToward(current_.x_mm, target_.x_mm,
                               kLinearSpeedMmPerSecond * dt_seconds);
    current_.z_mm = moveToward(current_.z_mm, target_.z_mm,
                               kLinearSpeedMmPerSecond * dt_seconds);
    current_.hip_pitch_offset_rad = moveToward(
        current_.hip_pitch_offset_rad, target_.hip_pitch_offset_rad,
        kHipSpeedRadPerSecond * dt_seconds);

    JointArray solved{};
    if (!solve(current_, solved)) {
        current_ = previous;
        target_ = previous;
        command = last_command_;
        return false;
    }

    last_command_ = solved;
    command = solved;
    return true;
}

float BodyPoseController::leftHipPitch() const {
    return reference_hip_pitch_[0] + current_.hip_pitch_offset_rad;
}

float BodyPoseController::rightHipPitch() const {
    return reference_hip_pitch_[1] + current_.hip_pitch_offset_rad;
}

}  // namespace tilt::wheel_base
