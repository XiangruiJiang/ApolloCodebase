// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 The Hong Kong University of Science and Technology (Guangzhou), Humanoid Computing & Learning Lab.

#pragma once

#include "src/world/world_snapshot.h"

#include <array>
#include <optional>

namespace decision {

/// Obstacle-aware heading produced by the local walk planner.
struct WalkPlan {
    double heading_deg;
};

/// Computes an obstacle-avoiding walk heading using grid A*.
///
/// All nearby players are obstacles unless `avoid_obstacles` is false. Speed is
/// scheduled by the caller; this function only chooses the heading.
WalkPlan plan_walk(
    const std::array<double, 2>& self_pos,
    const std::array<double, 2>& target_pos,
    const world::WorldSnapshot& snapshot,
    int self_player_number,
    std::optional<double> opponent_x_threshold = std::nullopt,
    bool avoid_field_boundaries = true,
    bool avoid_obstacles = true);

}  // namespace decision
