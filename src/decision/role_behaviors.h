// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 The Hong Kong University of Science and Technology (Guangzhou), Humanoid Computing & Learning Lab.

#pragma once

#include "src/decision/blackboard.h"
#include "src/decision/high_level_command.h"
#include "src/decision/role_manager.h"
#include "src/world/world_snapshot.h"

#include <optional>

namespace decision {

/// Interface for role-specific command generation.
class RoleBehavior {
public:
    virtual ~RoleBehavior() = default;
    virtual bool matches(const Blackboard& blackboard) const = 0;
    virtual HighLevelCommand make_command(
        const world::WorldSnapshot& snapshot,
        const Blackboard& blackboard) const = 0;
};

/// Persistent attacker state carried between decision cycles.
struct APState {
    bool dribble_ready{false};
    bool set_play_released{false};
    double previous_ball_distance{0.0};
};

/// Generates the active-player command and set-play handoff state.
class APBehavior final {
public:
    bool matches(const Blackboard& blackboard) const;
    HighLevelCommand make_command(
        const world::WorldSnapshot& snapshot,
        const Blackboard& blackboard,
        RoleManager& role_manager) const;
    void reset_state() const { state_ = {}; }
private:
    mutable APState state_;
};

/// Walk-to-formation behavior shared by CBM, ST, CBL, CBR, and CDM.
///
/// The defensive variant clips the planner against opponents already past the
/// current ball line.
class SimpleRoleBehavior final : public RoleBehavior {
public:
    SimpleRoleBehavior(int role_id, bool defensive_opponent_clip)
        : role_id_(role_id), defensive_opponent_clip_(defensive_opponent_clip) {}
    bool matches(const Blackboard& blackboard) const override;
    HighLevelCommand make_command(
        const world::WorldSnapshot& snapshot,
        const Blackboard& blackboard) const override;
private:
    int role_id_;
    bool defensive_opponent_clip_;
    mutable APState relay_state_;
};

/// Generates goalkeeper positioning and goal-kick commands.
class GKBehavior final : public RoleBehavior {
public:
    bool matches(const Blackboard& blackboard) const override;
    HighLevelCommand make_command(
        const world::WorldSnapshot& snapshot,
        const Blackboard& blackboard) const override;
    void reset_state() const {}
};

/// Clears persistent state owned by all role behavior instances.
void reset_role_behavior_state();

int current_role_from_blackboard(const Blackboard& blackboard);

/// Selects the behavior matching the current role, if one is available.
std::optional<HighLevelCommand> select_role_behavior(
    const world::WorldSnapshot& snapshot,
    const Blackboard& blackboard,
    RoleManager& role_manager);

}  // namespace decision
