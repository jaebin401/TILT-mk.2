#include "tilt_kinematics.h"

#include <cmath>
#include <cstdint>

namespace tilt {
namespace {

// C++17 does not make std::sqrt/std::atan2 constexpr. These literals are the
// compile-time evaluations of the L2/L3 + ANKLE_FIXED reduction from the spec.
constexpr float L_EFF_MM = 59.495315956686106f;
constexpr float PHI_RAD = -0.05751861302440288f;
constexpr float REACH_CLAMP_EPS_MM = 0.001f;
constexpr float PI_RAD = 3.14159265358979323846f;
constexpr float TWO_PI_RAD = 2.0f * PI_RAD;

}  // namespace

Vec3 fk_foot(Leg leg, const float theta[3]) {
    const float theta1 = theta[0];
    const float theta2 = theta[1];
    const float theta3 = theta[2];

    // Keep the three physical segments explicit. FK must not use IK's
    // effective-link reduction so the two calculations remain independent.
    const float thigh_abs = theta2;
    const float calf_abs = theta2 + theta3 + KNEE_OFFSET_RAD;
    const float foot_abs = calf_abs + ANKLE_FIXED_RAD;

    const float u =
        THIGH_LENGTH_MM * std::sin(-thigh_abs) +
        CALF_LENGTH_MM * std::sin(-calf_abs) +
        FOOT_LENGTH_MM * std::sin(-foot_abs);
    const float v =
        -THIGH_LENGTH_MM * std::cos(thigh_abs) -
        CALF_LENGTH_MM * std::cos(calf_abs) -
        FOOT_LENGTH_MM * std::cos(foot_abs);

    const float hip_y = leg == Leg::LEFT ? Y_HIP_MM : -Y_HIP_MM;
    return {
        u * std::cos(theta1),
        u * std::sin(theta1) + hip_y,
        v,
    };
}

IkResult ik_foot(Leg leg, const Vec3& foot_target, float theta1_hint) {
    if (!std::isfinite(theta1_hint)) {
        theta1_hint = 0.0f;
    }
    if (!std::isfinite(foot_target.x) || !std::isfinite(foot_target.y) ||
        !std::isfinite(foot_target.z)) {
        return {{theta1_hint, 0.0f, 0.0f}, false};
    }

    const float hip_y = leg == Leg::LEFT ? Y_HIP_MM : -Y_HIP_MM;
    const float p_x = foot_target.x;
    const float p_y = foot_target.y - hip_y;
    float p_z = foot_target.z;
    const float radial_magnitude = std::sqrt(p_x * p_x + p_y * p_y);

    float theta1 = theta1_hint;
    float radial = 0.0f;
    if (radial_magnitude >= YAW_SINGULARITY_EPS_MM) {
        const float yaw_from_target = std::atan2(p_y, p_x);
        const float opposite_yaw = yaw_from_target >= 0.0f
                                       ? yaw_from_target - PI_RAD
                                       : yaw_from_target + PI_RAD;
        const float direct_difference = std::fabs(std::remainder(
            yaw_from_target - theta1_hint, TWO_PI_RAD));
        const float opposite_difference = std::fabs(std::remainder(
            opposite_yaw - theta1_hint, TWO_PI_RAD));
        theta1 = opposite_difference < direct_difference ? opposite_yaw
                                                           : yaw_from_target;
        radial = p_x * std::cos(theta1) + p_y * std::sin(theta1);
    } else {
        // The target lies below the yaw axis; preserve the caller's yaw.
        radial = p_x * std::cos(theta1) + p_y * std::sin(theta1);
    }

    float distance = std::sqrt(radial * radial + p_z * p_z);
    const float maximum_reach = THIGH_LENGTH_MM + L_EFF_MM;
    const float minimum_reach = std::fabs(THIGH_LENGTH_MM - L_EFF_MM);
    const bool reachable = distance <= maximum_reach && distance >= minimum_reach;

    float solved_distance = distance;
    if (distance > maximum_reach) {
        solved_distance = maximum_reach - REACH_CLAMP_EPS_MM;
    } else if (distance < minimum_reach) {
        solved_distance = minimum_reach + REACH_CLAMP_EPS_MM;
    }

    if (solved_distance != distance) {
        if (distance > 0.0f) {
            const float scale = solved_distance / distance;
            radial *= scale;
            p_z *= scale;
        } else {
            radial = 0.0f;
            p_z = -solved_distance;
        }
        distance = solved_distance;
    }

    float cosine_psi =
        (distance * distance - THIGH_LENGTH_MM * THIGH_LENGTH_MM -
         L_EFF_MM * L_EFF_MM) /
        (2.0f * THIGH_LENGTH_MM * L_EFF_MM);
    if (cosine_psi < -1.0f) {
        cosine_psi = -1.0f;
    } else if (cosine_psi > 1.0f) {
        cosine_psi = 1.0f;
    }

    const float psi = std::acos(cosine_psi);
    const float alpha = std::atan2(-radial, -p_z);
    const float beta = std::atan2(L_EFF_MM * std::sin(psi),
                                  THIGH_LENGTH_MM + L_EFF_MM * std::cos(psi));

    return {{
                theta1,
                alpha - beta,
                psi - KNEE_OFFSET_RAD - PHI_RAD,
            },
            reachable};
}

bool clamp_to_limits(Leg leg, float theta[3]) {
    if (theta == nullptr) {
        return false;
    }

    const std::uint8_t leg_index = static_cast<std::uint8_t>(leg);
    if (leg_index > static_cast<std::uint8_t>(Leg::RIGHT)) {
        return false;
    }

    bool unchanged = true;
    for (std::uint8_t joint = 0; joint < 3; ++joint) {
        const JointLimit& limit = JOINT_LIMIT[leg_index][joint];
        if (!std::isfinite(theta[joint])) {
            if (0.0f < limit.minimum_rad) {
                theta[joint] = limit.minimum_rad;
            } else if (0.0f > limit.maximum_rad) {
                theta[joint] = limit.maximum_rad;
            } else {
                theta[joint] = 0.0f;
            }
            unchanged = false;
        } else if (theta[joint] < limit.minimum_rad) {
            theta[joint] = limit.minimum_rad;
            unchanged = false;
        } else if (theta[joint] > limit.maximum_rad) {
            theta[joint] = limit.maximum_rad;
            unchanged = false;
        }
    }
    return unchanged;
}

}  // namespace tilt
