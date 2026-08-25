// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 The Hong Kong University of Science and Technology (Guangzhou), Humanoid Computing & Learning Lab.

#pragma once

#include "src/decision/behavior_tree.h"
#include "src/decision/blackboard.h"
#include "src/decision/high_level_command.h"
#include "src/decision/role_manager.h"
#include "src/world/world_snapshot.h"

namespace decision {

/// Owns persistent decision state and evaluates one command per world snapshot.
class DecisionManager {
public:
    DecisionManager();

    HighLevelCommand decide(const world::WorldSnapshot& snapshot);
    const Blackboard& blackboard() const;

private:
    Blackboard blackboard_;
    RoleManager role_manager_;
    BehaviorTree behavior_tree_;
};

}  // namespace decision
