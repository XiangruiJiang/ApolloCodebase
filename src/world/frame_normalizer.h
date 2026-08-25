// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 The Hong Kong University of Science and Technology (Guangzhou), Humanoid Computing & Learning Lab.

#pragma once

#include <array>

namespace world {

/// Cartesian 3-vector, with the concrete frame documented by each use site.
using Vec3 = std::array<double, 3>;
/// Quaternion stored in `(w, x, y, z)` order.
using QuaternionWxyz = std::array<double, 4>;

/// Converts simulator poses into the canonical frame with own goal at `-x`.
class FrameNormalizer {
public:
    /// Normalizes a simulator-frame position without changing its units.
    static Vec3 normalize_position(const Vec3& simulator_position_m, bool is_left_team);
    /// Normalizes orientation into the canonical team frame.
    static QuaternionWxyz normalize_quaternion_wxyz(const QuaternionWxyz& simulator_quaternion_wxyz, bool is_left_team);
    static double yaw_deg_from_quaternion_wxyz(const QuaternionWxyz& quaternion_wxyz);
};

}  // namespace world
