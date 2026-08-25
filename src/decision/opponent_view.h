// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 The Hong Kong University of Science and Technology (Guangzhou), Humanoid Computing & Learning Lab.

#pragma once

#include "src/math/math_utils.h"
#include "src/world/world_snapshot.h"

#include <array>
#include <vector>

namespace decision {

inline void append_distinct_opponent_position(
    std::vector<std::array<double, 2>>& positions,
    const std::array<double, 2>& position,
    double dedup_distance_m) {
    for (const auto& existing : positions) {
        if (math::planar_dist(existing, position) < dedup_distance_m) {
            return;
        }
    }
    positions.push_back(position);
}

/// Merges recent local and team-shared opponents, removing near duplicates.
inline std::vector<std::array<double, 2>> collect_known_opponent_positions(
    const world::WorldSnapshot& snapshot,
    double dedup_distance_m = 1.0,
    double recent_timeout_s = 2.0) {
    std::vector<std::array<double, 2>> positions;
    positions.reserve(snapshot.opponents.size() + snapshot.shared_opponents.size());

    for (const auto& opponent : snapshot.opponents) {
        if (!opponent.seen && (snapshot.server_time - opponent.last_seen_time) > recent_timeout_s) {
            continue;
        }
        append_distinct_opponent_position(
            positions,
            {opponent.position_m[0], opponent.position_m[1]},
            dedup_distance_m);
    }

    for (const auto& opponent : snapshot.shared_opponents) {
        if (!opponent.seen && (snapshot.server_time - opponent.last_seen_time) > recent_timeout_s) {
            continue;
        }
        append_distinct_opponent_position(
            positions,
            {opponent.position_m[0], opponent.position_m[1]},
            dedup_distance_m);
    }

    return positions;
}

}  // namespace decision
