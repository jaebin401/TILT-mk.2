#include "tilt_kinematics.h"

#include <cmath>
#include <cstdint>

#include "tilt/kinematics/RobotModel.h"

namespace tilt {
namespace {

constexpr float PI_RAD = 3.14159265358979323846f;
constexpr float REACH_CLAMP_EPS_MM = 0.001f;

Mat3 identity() {
    return {{{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}}};
}

Mat3 multiply(const Mat3& a, const Mat3& b) {
    Mat3 out{};
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            out.m[r][c] = a.m[r][0] * b.m[0][c] + a.m[r][1] * b.m[1][c] +
                          a.m[r][2] * b.m[2][c];
        }
    }
    return out;
}

Mat3 from_array(const float m[3][3]) {
    Mat3 out{};
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) out.m[r][c] = m[r][c];
    }
    return out;
}

Vec3 rotate(const Mat3& R, const float v[3]) {
    return {
        R.m[0][0] * v[0] + R.m[0][1] * v[1] + R.m[0][2] * v[2],
        R.m[1][0] * v[0] + R.m[1][1] * v[1] + R.m[1][2] * v[2],
        R.m[2][0] * v[0] + R.m[2][1] * v[1] + R.m[2][2] * v[2],
    };
}

// Rodrigues rotation about a unit axis.
Mat3 axis_angle(const float axis[3], float angle) {
    const float x = axis[0];
    const float y = axis[1];
    const float z = axis[2];
    const float c = std::cos(angle);
    const float s = std::sin(angle);
    const float t = 1.0f - c;
    return {{
        {t * x * x + c, t * x * y - s * z, t * x * z + s * y},
        {t * x * y + s * z, t * y * y + c, t * y * z - s * x},
        {t * x * z - s * y, t * y * z + s * x, t * z * z + c},
    }};
}

// child = parent * Translate(origin) * rotation * Rot(axis, q)
Pose apply_joint(const Pose& parent, const model::JointSpec& joint, float q) {
    const Vec3 offset = rotate(parent.R, joint.origin_mm);
    Pose child{};
    child.p = {parent.p.x + offset.x, parent.p.y + offset.y,
               parent.p.z + offset.z};
    child.R = multiply(parent.R, from_array(joint.rotation));
    if (q != 0.0f) child.R = multiply(child.R, axis_angle(joint.axis, q));
    return child;
}

float wrap_pi(float angle) {
    return std::remainder(angle, 2.0f * PI_RAD);
}

}  // namespace

Pose base_pose() {
    return {identity(), {0.0f, 0.0f, 0.0f}};
}

Pose joint_child_pose(const Pose& parent, const model::JointSpec& joint,
                      float q) {
    return apply_joint(parent, joint, q);
}

Vec3 transform_point(const Pose& pose, const Vec3& local) {
    const float v[3] = {local.x, local.y, local.z};
    const Vec3 r = rotate(pose.R, v);
    return {pose.p.x + r.x, pose.p.y + r.y, pose.p.z + r.z};
}

LegFrames fk_leg(Leg leg, const float theta[3]) {
    const int i = static_cast<int>(leg);
    const Pose base{identity(), {0.0f, 0.0f, 0.0f}};
    LegFrames frames{};
    frames.hip = apply_joint(base, model::LEG_JOINT[i][0], theta[0]);
    frames.thigh = apply_joint(frames.hip, model::LEG_JOINT[i][1], theta[1]);
    frames.calf = apply_joint(frames.thigh, model::LEG_JOINT[i][2], theta[2]);
    frames.sole = apply_joint(frames.calf, model::SOLE_FIXED[i], 0.0f);
    return frames;
}

Vec3 fk_foot(Leg leg, const float theta[3]) {
    return fk_leg(leg, theta).sole.p;
}

float sole_pitch_rad(const float theta[3]) {
    return model::SOLE_PITCH_AT_ZERO_RAD + theta[1] + theta[2];
}

IkResult ik_foot(Leg leg, const Vec3& sole_target) {
    const int i = static_cast<int>(leg);
    if (!std::isfinite(sole_target.x) || !std::isfinite(sole_target.y) ||
        !std::isfinite(sole_target.z)) {
        return {{0.0f, 0.0f, 0.0f}, false};
    }
    bool reachable = true;

    // Target relative to the hip point H.
    const float rx = sole_target.x - model::HIP_POINT_MM[i][0];
    const float ry = sole_target.y - model::HIP_POINT_MM[i][1];
    const float rz = sole_target.z - model::HIP_POINT_MM[i][2];

    // Frontal plane: in the roll frame the sole sits at (y', z') = (d, zp)
    // with a constant lateral offset d, so |(ry, rz)| = |(d, zp)|.
    const float d = model::SOLE_LATERAL_MM[i];
    float rho = std::sqrt(ry * ry + rz * rz);
    const float rho_min = std::fabs(d) + REACH_CLAMP_EPS_MM;
    if (rho < rho_min) {
        rho = rho_min;
        reachable = false;
    }
    const float zp = -std::sqrt(rho * rho - d * d);  // Foot below the hip.
    const float roll = wrap_pi(std::atan2(rz, ry) - std::atan2(zp, d));

    // Sagittal plane: two links rotating about +y (Ry rotates (x, z) by -q).
    const float tx = model::THIGH_XZ_MM[i][0];
    const float tz = model::THIGH_XZ_MM[i][1];
    const float sx = model::SHANK_XZ_MM[i][0];
    const float sz = model::SHANK_XZ_MM[i][1];
    const float l1 = std::sqrt(tx * tx + tz * tz);
    const float l2 = std::sqrt(sx * sx + sz * sz);

    float px = rx;
    float pz = zp;
    float dist = std::sqrt(px * px + pz * pz);
    const float max_reach = l1 + l2 - REACH_CLAMP_EPS_MM;
    const float min_reach = std::fabs(l1 - l2) + REACH_CLAMP_EPS_MM;
    float clamped = dist;
    if (dist > max_reach) clamped = max_reach;
    if (dist < min_reach) clamped = min_reach;
    if (clamped != dist) {
        reachable = false;
        if (dist > 0.0f) {
            px *= clamped / dist;
            pz *= clamped / dist;
        } else {
            px = 0.0f;
            pz = -clamped;
        }
        dist = clamped;
    }

    float cosine = (dist * dist - l1 * l1 - l2 * l2) / (2.0f * l1 * l2);
    if (cosine > 1.0f) cosine = 1.0f;
    if (cosine < -1.0f) cosine = -1.0f;
    const float angle_thigh = std::atan2(tz, tx);
    const float angle_shank = std::atan2(sz, sx);
    const float knee = angle_shank - angle_thigh + std::acos(cosine);

    // w = thigh + Ry(knee) * shank ; Ry(hip) * w must point at (px, pz).
    const float ck = std::cos(knee);
    const float sk = std::sin(knee);
    const float wx = tx + (sx * ck + sz * sk);
    const float wz = tz + (-sx * sk + sz * ck);
    const float hip = wrap_pi(std::atan2(wz, wx) - std::atan2(pz, px));

    return {{roll, hip, knee}, reachable};
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
