// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 The Hong Kong University of Science and Technology (Guangzhou), Humanoid Computing & Learning Lab.

#pragma once

#include "src/decision/field_geometry.h"
#include "src/decision/formation.h"
#include "src/world/world_snapshot.h"

#include <array>
#include <tuple>

namespace decision {

/// One player's assigned role and canonical-field target.
struct RoleAssignment {
    int player_number{0};
    int role_id{0};
    std::array<double, 2> role_position_m{0.0, 0.0};
};

/// Player state used by the free-role assignment solver.
struct PlayerRoleCandidate {
    int player_number{0};
    std::array<double, 2> position_m{0.0, 0.0};
    bool fallen{false};
    int comm_role{-1};
};

/// Assigns stable tactical roles to active teammates each cycle.
class RoleManager {
public:
    static constexpr int ROLE_GK = 0;
    static constexpr int ROLE_CBL = 1;
    static constexpr int ROLE_CBR = 2;
    static constexpr int ROLE_CDM = 3;
    static constexpr int ROLE_CBM = 4;
    static constexpr int ROLE_ST = 5;
    static constexpr int ROLE_AP = 6;

    // Sized for player numbers 1..7 inclusive (index 0 unused). The team is
    // 7v7 throughout (WorldState builds 7 teammate slots, TeamCommCodec and
    // TeamCommManager clamp senders to 1..7).
    static constexpr std::size_t kPreviousRoleSlots = 8;
    using PreviousRoleByPlayer = std::array<int, kPreviousRoleSlots>;

    using RoleResult = std::tuple<int, std::array<double, 2>>;

    explicit RoleManager(
        double field_length_m = field_geometry::kActualFieldLengthM,
        double field_width_m = field_geometry::kActualFieldWidthM);

    /// Assigns all available players while preserving useful prior assignments.
    std::vector<RoleAssignment> assign(const world::WorldSnapshot& snapshot) const;

    /// Returns the calling player's role id and formation position.
    RoleResult get_role(const world::WorldSnapshot& snapshot) const;

    /// Marks a player as having touched the ball in the current set play.
    ///
    /// Role assignment skips that player for AP until the mode changes.
    void mark_self_set_play_pushed(int self_player_number,
                                    const world::WorldSnapshot& snapshot);
    bool is_self_set_play_pushed(int self_player_number,
                                 const world::WorldSnapshot& snapshot) const;

private:
    double field_length_m_{field_geometry::kActualFieldLengthM};
    double field_width_m_{field_geometry::kActualFieldWidthM};
    // Last tick's role assignment, indexed by player_number (1..7). Used to
    // bias this tick's free-role assignment toward the previous outcome and
    // prevent 1-tick role flips when two players have nearly equal cost.
    mutable PreviousRoleByPlayer previous_role_by_player_;
    // Per-agent latch: the player number that already pushed the ball in the
    // current our-kick set play and must not be re-selected as AP. Cleared the
    // next time the world leaves OurKick.
    mutable int pushed_set_play_player_{-1};
    mutable world::PlayMode pushed_set_play_mode{world::PlayMode::NotInitialized};
};

/// Solves the remaining free-player to free-role assignment.
std::vector<RoleAssignment> assign_remaining_players(
    const Formation::RolePositions& formation_positions,
    const std::vector<PlayerRoleCandidate>& available_players,
    const std::vector<int>& remaining_roles,
    const RoleManager::PreviousRoleByPlayer& previous_role_by_player);

}  // namespace decision
