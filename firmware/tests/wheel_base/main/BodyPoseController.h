#pragma once

#include <array>

#include "tilt_kinematics.h"

namespace tilt::wheel_base {

using JointArray = std::array<float, NUM_JOINTS>;

struct BodyTarget {
    float x_mm = 0.0f;
    float z_mm = 0.0f;
    float hip_pitch_offset_rad = 0.0f;
};

class BodyPoseController {
public:
    void captureReference(const JointArray& pose);
    bool initialized() const { return initialized_; }

    void nudgePosition(float x_mm, float z_mm);
    void nudgeHipPitch(float radians);
    void resetTarget();

    // Advances the body target at bounded rates and solves both legs. On an
    // invalid target the attempted step is cancelled and the last valid joint
    // command is returned.
    bool update(float dt_seconds, JointArray& command);

    const BodyTarget& current() const { return current_; }
    const BodyTarget& target() const { return target_; }
    float leftHipPitch() const;
    float rightHipPitch() const;

private:
    bool solve(const BodyTarget& body, JointArray& command) const;

    std::array<Vec3, 2> reference_foot_{};
    std::array<float, 2> reference_hip_roll_{};
    std::array<float, 2> reference_hip_pitch_{};
    JointArray last_command_{};
    BodyTarget current_{};
    BodyTarget target_{};
    bool initialized_ = false;
};

}  // namespace tilt::wheel_base
