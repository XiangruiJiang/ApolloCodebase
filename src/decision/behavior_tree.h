// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 The Hong Kong University of Science and Technology (Guangzhou), Humanoid Computing & Learning Lab.

#pragma once

#include "src/decision/blackboard.h"
#include "src/decision/high_level_command.h"
#include "src/decision/role_manager.h"
#include "src/world/world_snapshot.h"

namespace decision {

/// Evaluates match state, set plays, and role behavior into one command.
class BehaviorTree {
public:
    HighLevelCommand evaluate(
        const world::WorldSnapshot& snapshot,
        Blackboard& blackboard,
        RoleManager& role_manager) const;
};

}  // namespace decision
