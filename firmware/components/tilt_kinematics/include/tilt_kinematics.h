#pragma once

// WARNING(mk.2): mk.1 hip yaw 체인 기준. Y_HIP_MM 의미도 바뀌어 y 결과가 틀림.
// roll IK로 재작성하기 전까지 사용 금지.

#include "tilt_config.h"

namespace tilt {

enum class Leg { LEFT = 0, RIGHT = 1 };

struct Vec3 {
    float x;
    float y;
    float z;
};

struct IkResult {
    float theta[3];
    bool reachable;
};

// Forward kinematics for validation and diagnostics; not part of the control loop.
Vec3 fk_foot(Leg leg, const float theta[3]);

// Closed-form inverse kinematics. theta1_hint preserves yaw continuity around
// the hip-yaw singularity and across the equivalent opposite-radial solution.
IkResult ik_foot(Leg leg, const Vec3& foot_target, float theta1_hint);

// Clamps in place to the logical joint limits for the selected leg.
bool clamp_to_limits(Leg leg, float theta[3]);

}  // namespace tilt
