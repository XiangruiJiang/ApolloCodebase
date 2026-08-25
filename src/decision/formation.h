// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 The Hong Kong University of Science and Technology (Guangzhou), Humanoid Computing & Learning Lab.

#pragma once

#include <array>
#include <vector>

#include "src/world/play_mode.h"

namespace decision {

/// Inputs used to construct one seven-role formation.
struct FormationContext {
    std::array<double, 2> ball_position_m{0.0, 0.0};
    world::PlayMode play_mode{world::PlayMode::PlayOn};
    double field_length_m{55.0};
    double field_width_m{36.0};
};

/// Computes fixed set-play or ball-relative open-play role positions.
class Formation {
public:
    using RolePosition = std::array<double, 2>;
    using RolePositions = std::array<RolePosition, 7>;

    RolePositions compute(const FormationContext& ctx) const;
};

}  // namespace decision
