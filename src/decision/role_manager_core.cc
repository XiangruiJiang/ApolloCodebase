// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 The Hong Kong University of Science and Technology (Guangzhou), Humanoid Computing & Learning Lab.

#include "src/decision/role_manager.h"

#include "src/math/math_utils.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>

namespace decision {

namespace {

std::array<double, 2> fallback_position_for_player(int player_number) {
    return field_geometry::player_fallback_position(player_number);
}

bool has_known_position(const world::WorldSnapshot& snapshot, const world::PlayerObservation& player) {
    return player.player_number == snapshot.player_number || player.seen || player.last_seen_time >= 0.0;
}

std::array<double, 2> effective_position(
    const world::WorldSnapshot& snapshot,
    const world::PlayerObservation& player) {
    if (has_known_position(snapshot, player)) {
        if (player.player_number == snapshot.player_number) {
            return {snapshot.self.position_m[0], snapshot.self.position_m[1]};
        }
        return {player.position_m[0], player.position_m[1]};
    }
    return fallback_position_for_player(player.player_number);
}

std::vector<PlayerRoleCandidate> make_teammate_candidates(const world::WorldSnapshot& snapshot) {
    std::vector<PlayerRoleCandidate> candidates;
    candidates.reserve(snapshot.teammates.size());
    for (const auto& teammate : snapshot.teammates) {
        if (teammate.player_number <= 0) {
            continue;
        }
        PlayerRoleCandidate candidate;
        candidate.player_number = teammate.player_number;
        candidate.position_m = effective_position(snapshot, teammate);
        candidate.fallen = teammate.fallen;
        candidate.comm_role = teammate.comm_role;
        candidates.push_back(candidate);
    }
    return candidates;
}

int select_goalkeeper(const std::vector<PlayerRoleCandidate>& teammates) {
    // Player 1 is the designated goalkeeper, but if it is fallen we promote
    // the lowest-numbered non-fallen teammate to keep the goal defended while
    // player 1 recovers. Falls back to player 1 (or the first teammate) only
    // when no non-fallen candidate exists at all.
    auto find_player = [&](int player_number) {
        return std::find_if(
            teammates.begin(),
            teammates.end(),
            [&](const PlayerRoleCandidate& p) { return p.player_number == player_number; });
    };

    const auto p1 = find_player(1);
    if (p1 != teammates.end() && !p1->fallen) {
        return p1->player_number;
    }

    int best = -1;
    for (const auto& player : teammates) {
        if (player.fallen) {
            continue;
        }
        if (best < 0 || player.player_number < best) {
            best = player.player_number;
        }
    }
    if (best > 0) {
        return best;
    }

    if (p1 != teammates.end()) {
        return p1->player_number;
    }
    return teammates.empty() ? 1 : teammates.front().player_number;
}

// Comm-aware AP tiebreak margin. If a teammate is currently broadcasting
// AP and is within this many meters of the locally-best candidate's distance
// to the ball, defer to them. Prevents AP from oscillating between two
// robots whose teammate-position observations straddle each other.
constexpr double kAPSwitchMarginM = 0.5;

std::optional<int> select_ap(
    const std::vector<PlayerRoleCandidate>& teammates,
    int goalkeeper_player_number,
    int self_player_number,
    int self_previous_role,
    const std::array<double, 2>& ball_position_m,
    int excluded_player_number = -1) {
    int best_player = -1;
    double best_distance = std::numeric_limits<double>::infinity();
    double self_distance = std::numeric_limits<double>::infinity();

    for (const auto& player : teammates) {
        if (player.player_number == goalkeeper_player_number || player.fallen) {
            continue;
        }
        if (player.player_number == excluded_player_number) {
            continue;
        }
        const double distance = math::sq_dist2(player.position_m, ball_position_m);
        if (distance < best_distance) {
            best_distance = distance;
            best_player = player.player_number;
        }
        if (player.player_number == self_player_number) {
            self_distance = distance;
        }
    }

    if (best_player <= 0) {
        return std::nullopt;
    }

    const double best_linear_distance = std::sqrt(best_distance);

    // Self-hysteresis: if I was AP last tick and I am still within margin of
    // the locally-best candidate, retain AP. Prevents one-tick flapping when
    // two robots straddle the equidistant line.
    if (self_previous_role == RoleManager::ROLE_AP &&
        self_player_number != goalkeeper_player_number &&
        std::isfinite(self_distance)) {
        const double self_linear_distance = std::sqrt(self_distance);
        if (self_linear_distance <= best_linear_distance + kAPSwitchMarginM) {
            return self_player_number;
        }
    }

    // Comm-aware tiebreak: among ALL teammates currently broadcasting AP
    // (excluding the GK and fallen players), pick the closest one. Defer to
    // them only if their distance is within margin of the locally-best
    // candidate. Fixes a previous one-iteration loop bug that ignored every
    // AP claimant after the first.
    int comm_ap_player = -1;
    double comm_ap_distance = std::numeric_limits<double>::infinity();
    for (const auto& player : teammates) {
        if (player.comm_role != RoleManager::ROLE_AP) {
            continue;
        }
        if (player.player_number == goalkeeper_player_number || player.fallen) {
            continue;
        }
        const double d = math::sq_dist2(player.position_m, ball_position_m);
        if (d < comm_ap_distance) {
            comm_ap_distance = d;
            comm_ap_player = player.player_number;
        }
    }
    if (comm_ap_player > 0 && comm_ap_player != best_player) {
        const double comm_linear_distance = std::sqrt(comm_ap_distance);
        if (comm_linear_distance <= best_linear_distance + kAPSwitchMarginM) {
            return comm_ap_player;
        }
    }

    return best_player;
}

bool ap_should_use_formation_target(const world::WorldSnapshot& snapshot) {
    return snapshot.play_mode != world::PlayMode::PlayOn &&
           snapshot.play_mode_group != world::PlayModeGroup::OurKick;
}

std::vector<int> remaining_players(
    const std::vector<PlayerRoleCandidate>& teammates,
    int goalkeeper_player_number,
    int ap_player_number) {
    std::vector<int> players;
    for (const auto& teammate : teammates) {
        if (teammate.fallen) {
            continue;
        }
        if (teammate.player_number == goalkeeper_player_number || teammate.player_number == ap_player_number) {
            continue;
        }
        players.push_back(teammate.player_number);
    }
    return players;
}

}  // namespace

RoleManager::RoleManager(double field_length_m, double field_width_m)
    : field_length_m_(field_length_m),
      field_width_m_(field_width_m) {
    previous_role_by_player_.fill(-1);
}

void RoleManager::mark_self_set_play_pushed(int self_player_number,
                                             const world::WorldSnapshot& snapshot) {
    if (snapshot.play_mode_group == world::PlayModeGroup::OurKick) {
        pushed_set_play_player_ = self_player_number;
        pushed_set_play_mode = snapshot.play_mode;
    }
}

bool RoleManager::is_self_set_play_pushed(int self_player_number,
                                          const world::WorldSnapshot& snapshot) const {
    if (snapshot.play_mode_group != world::PlayModeGroup::OurKick) {
        pushed_set_play_player_ = -1;
        pushed_set_play_mode = world::PlayMode::NotInitialized;
        return false;
    }
    if (pushed_set_play_mode != snapshot.play_mode) {
        pushed_set_play_player_ = -1;
        pushed_set_play_mode = snapshot.play_mode;
    }
    return pushed_set_play_player_ == self_player_number;
}

std::vector<RoleAssignment> RoleManager::assign(
    const world::WorldSnapshot& snapshot) const {
    const std::array<double, 2> ball_position_m{
        snapshot.ball.position_m[0],
        snapshot.ball.position_m[1],
    };

    FormationContext ctx;
    ctx.ball_position_m = ball_position_m;
    ctx.play_mode = snapshot.play_mode;
    ctx.field_length_m = field_length_m_;
    ctx.field_width_m = field_width_m_;

    const Formation formation;
    const auto formation_positions = formation.compute(ctx);
    const std::vector<PlayerRoleCandidate> teammate_candidates = make_teammate_candidates(snapshot);

    std::vector<RoleAssignment> assignments;
    assignments.reserve(snapshot.teammates.size());
    for (const auto& teammate : snapshot.teammates) {
        assignments.push_back({teammate.player_number, -1, fallback_position_for_player(teammate.player_number)});
    }

    const int goalkeeper_player_number = select_goalkeeper(teammate_candidates);
    const auto self_pn_idx = static_cast<std::size_t>(snapshot.player_number);
    const int self_previous_role =
        (self_pn_idx < previous_role_by_player_.size())
            ? previous_role_by_player_[self_pn_idx]
            : -1;
    const int excluded_ap_player = is_self_set_play_pushed(snapshot.player_number, snapshot)
        ? snapshot.player_number
        : -1;
    const std::optional<int> ap_player_number = select_ap(
        teammate_candidates,
        goalkeeper_player_number,
        snapshot.player_number,
        self_previous_role,
        ball_position_m,
        excluded_ap_player);

    auto write_assignment = [&](int player_number, int role_id, const std::array<double, 2>& role_position_m) {
        if (player_number <= 0 || static_cast<std::size_t>(player_number) > assignments.size()) {
            throw std::invalid_argument("Player number out of range for role assignment");
        }
        auto& entry = assignments[static_cast<std::size_t>(player_number - 1)];
        entry.player_number = player_number;
        entry.role_id = role_id;
        entry.role_position_m = role_position_m;
    };

    write_assignment(goalkeeper_player_number, ROLE_GK, formation_positions[static_cast<std::size_t>(ROLE_GK)]);
    if (ap_player_number.has_value()) {
        const auto& ap_target = ap_should_use_formation_target(snapshot)
            ? formation_positions[static_cast<std::size_t>(ROLE_AP)]
            : ball_position_m;
        write_assignment(*ap_player_number, ROLE_AP, ap_target);
    }

    const std::vector<int> free_players =
        remaining_players(teammate_candidates, goalkeeper_player_number, ap_player_number.value_or(-1));
    std::vector<PlayerRoleCandidate> free_player_candidates;
    free_player_candidates.reserve(free_players.size());
    for (const auto& candidate : teammate_candidates) {
        if (std::find(free_players.begin(), free_players.end(), candidate.player_number) != free_players.end()) {
            free_player_candidates.push_back(candidate);
        }
    }

    const std::vector<int> remaining_roles{ROLE_CDM, ROLE_CBL, ROLE_CBR, ROLE_CBM, ROLE_ST};
    const std::vector<RoleAssignment> field_assignments = assign_remaining_players(
        formation_positions, free_player_candidates, remaining_roles, previous_role_by_player_);
    for (const auto& assignment : field_assignments) {
        write_assignment(assignment.player_number, assignment.role_id, assignment.role_position_m);
    }

    // Refresh per-tick role memory so next tick's permutation gets the sticky
    // bonus on the same pairings.
    previous_role_by_player_.fill(-1);
    for (const auto& assignment : assignments) {
        const auto pn = static_cast<std::size_t>(assignment.player_number);
        if (pn < previous_role_by_player_.size() && assignment.role_id >= 0) {
            previous_role_by_player_[pn] = assignment.role_id;
        }
    }

    return assignments;
}

RoleManager::RoleResult RoleManager::get_role(const world::WorldSnapshot& snapshot) const {
    const std::vector<RoleAssignment> assignments = assign(snapshot);
    const auto it = std::find_if(
        assignments.begin(),
        assignments.end(),
        [&](const RoleAssignment& assignment) { return assignment.player_number == snapshot.player_number; });
    if (it == assignments.end()) {
        throw std::invalid_argument("Snapshot player_number missing from teammate assignments");
    }

    return std::make_tuple(it->role_id, it->role_position_m);
}

}  // namespace decision
