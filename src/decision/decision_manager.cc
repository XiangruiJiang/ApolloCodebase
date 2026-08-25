// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 The Hong Kong University of Science and Technology (Guangzhou), Humanoid Computing & Learning Lab.

#include "src/decision/decision_manager.h"

namespace decision {

DecisionManager::DecisionManager() = default;

HighLevelCommand DecisionManager::decide(const world::WorldSnapshot& snapshot) {
    return behavior_tree_.evaluate(snapshot, blackboard_, role_manager_);
}

const Blackboard& DecisionManager::blackboard() const {
    return blackboard_;
}

}  // namespace decision
