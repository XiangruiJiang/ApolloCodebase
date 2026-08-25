// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 The Hong Kong University of Science and Technology (Guangzhou), Humanoid Computing & Learning Lab.

#include "src/world/frame_normalizer.h"

#include "src/math/math_utils.h"

namespace world {

Vec3 FrameNormalizer::normalize_position(const Vec3& simulator_position_m, bool is_left_team) {
    if (is_left_team) {
        return simulator_position_m;
    }
    return {-simulator_position_m[0], -simulator_position_m[1], simulator_position_m[2]};
}

QuaternionWxyz FrameNormalizer::normalize_quaternion_wxyz(const QuaternionWxyz& simulator_quaternion_wxyz, bool is_left_team) {
    if (is_left_team) {
        return simulator_quaternion_wxyz;
    }
    const QuaternionWxyz yaw180{0.0, 0.0, 0.0, 1.0};
    return math::quaternion_multiply(yaw180, simulator_quaternion_wxyz);
}

double FrameNormalizer::yaw_deg_from_quaternion_wxyz(const QuaternionWxyz& quaternion_wxyz) {
    const double w = quaternion_wxyz[0];
    const double x = quaternion_wxyz[1];
    const double y = quaternion_wxyz[2];
    const double z = quaternion_wxyz[3];
    const double siny_cosp = 2.0 * (w * z + x * y);
    const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    return math::normalize_deg(math::rad_to_deg(std::atan2(siny_cosp, cosy_cosp)));
}

}  // namespace world
