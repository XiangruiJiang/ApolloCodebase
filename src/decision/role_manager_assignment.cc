// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 The Hong Kong University of Science and Technology (Guangzhou), Humanoid Computing & Learning Lab.

#include "src/decision/role_manager.h"

#include "src/math/math_utils.h"

#include <algorithm>
#include <array>
#include <limits>
#include <numeric>
#include <vector>

namespace decision {

namespace {

// Hysteresis bonus (squared meters) applied when a player keeps the role it
// held last tick. Equivalent to ~0.5 m of distance pull; small enough that a
// genuinely closer slot still wins, large enough to break tie-induced flips.
constexpr double kStickyBonusSqM = 0.25;

// Upper bound on permutation count. 7v7 has ≤5 free outfielders (5! = 120).
// At 8 we are at 40 320 permutations — still ~0.5 ms at 50 Hz, marginal but
// safe. Above 8 the factorial explodes and this approach must be replaced
// with an O(N³) LAPJV/Hungarian solver.
constexpr std::size_t kMaxPermutationSlots = 8;

double evaluate_permutation(
    const Formation::RolePositions& formation_positions,
    const std::vector<PlayerRoleCandidate>& available_players,
    const std::vector<int>& remaining_roles,
    const RoleManager::PreviousRoleByPlayer& previous_role_by_player,
    const std::array<std::size_t, kMaxPermutationSlots>& player_perm,
    std::size_t pair_count) {
    double total = 0.0;
    for (std::size_t r = 0; r < pair_count; ++r) {
        const auto& player = available_players[player_perm[r]];
        const int role_id = remaining_roles[r];
        const auto& role_pos = formation_positions[static_cast<std::size_t>(role_id)];
        double pair_cost = math::sq_dist2(player.position_m, role_pos);
        const auto pn = static_cast<std::size_t>(player.player_number);
        if (pn < previous_role_by_player.size() &&
            previous_role_by_player[pn] == role_id) {
            pair_cost -= kStickyBonusSqM;
        }
        total += pair_cost;
    }
    return total;
}

}  // namespace

std::vector<RoleAssignment> assign_remaining_players(
    const Formation::RolePositions& formation_positions,
    const std::vector<PlayerRoleCandidate>& available_players,
    const std::vector<int>& remaining_roles,
    const RoleManager::PreviousRoleByPlayer& previous_role_by_player) {
    const std::size_t n_players = available_players.size();
    const std::size_t n_roles = remaining_roles.size();
    if (n_players == 0 || n_roles == 0) {
        return {};
    }

    // When the free-player count exceeds the permutation budget, restrict the
    // permutation to the players closest to any remaining role; leftover
    // players still get a sensible role (the closest remaining_role's slot)
    // so we never return role_id == -1 for an active teammate.
    std::vector<std::size_t> selected_indices;
    selected_indices.reserve(n_players);
    for (std::size_t i = 0; i < n_players; ++i) {
        selected_indices.push_back(i);
    }

    auto min_sq_dist_to_any_role = [&](std::size_t idx) {
        double best = std::numeric_limits<double>::infinity();
        const auto& pos = available_players[idx].position_m;
        for (int role_id : remaining_roles) {
            const auto& target = formation_positions[static_cast<std::size_t>(role_id)];
            best = std::min(best, math::sq_dist2(pos, target));
        }
        return best;
    };

    if (n_players > kMaxPermutationSlots) {
        std::sort(
            selected_indices.begin(),
            selected_indices.end(),
            [&](std::size_t a, std::size_t b) {
                return min_sq_dist_to_any_role(a) < min_sq_dist_to_any_role(b);
            });
    }

    const std::size_t perm_size = std::min(n_players, kMaxPermutationSlots);
    std::vector<PlayerRoleCandidate> perm_players;
    perm_players.reserve(perm_size);
    for (std::size_t i = 0; i < perm_size; ++i) {
        perm_players.push_back(available_players[selected_indices[i]]);
    }

    std::array<std::size_t, kMaxPermutationSlots> player_perm{};
    std::iota(player_perm.begin(), player_perm.begin() + perm_size, std::size_t{0});

    const std::size_t pair_count = std::min(perm_size, n_roles);

    auto best_perm = player_perm;
    double best_cost = evaluate_permutation(
        formation_positions, perm_players, remaining_roles,
        previous_role_by_player, player_perm, pair_count);

    while (std::next_permutation(
        player_perm.begin(), player_perm.begin() + perm_size)) {
        const double cost = evaluate_permutation(
            formation_positions, perm_players, remaining_roles,
            previous_role_by_player, player_perm, pair_count);
        if (cost < best_cost) {
            best_cost = cost;
            best_perm = player_perm;
        }
    }

    std::vector<RoleAssignment> result;
    result.reserve(n_players);
    for (std::size_t r = 0; r < pair_count; ++r) {
        const auto& player = perm_players[best_perm[r]];
        const int role_id = remaining_roles[r];
        result.push_back({
            player.player_number,
            role_id,
            formation_positions[static_cast<std::size_t>(role_id)],
        });
    }

    // Cover any leftover players (when n_players exceeds the permutation
    // budget OR when there are fewer roles than players). Each leftover takes
    // the closest remaining_role's slot — they will cluster at that slot, but
    // path-planner avoidance keeps them from colliding, and crucially we
    // never leave an active teammate with role_id == -1.
    for (std::size_t i = pair_count; i < n_players; ++i) {
        const std::size_t idx = (n_players > kMaxPermutationSlots)
            ? selected_indices[i]
            : i;
        const auto& player = available_players[idx];
        int closest_role = remaining_roles.front();
        double closest_d = std::numeric_limits<double>::infinity();
        for (int role_id : remaining_roles) {
            const auto& target = formation_positions[static_cast<std::size_t>(role_id)];
            const double d = math::sq_dist2(player.position_m, target);
            if (d < closest_d) {
                closest_d = d;
                closest_role = role_id;
            }
        }
        result.push_back({
            player.player_number,
            closest_role,
            formation_positions[static_cast<std::size_t>(closest_role)],
        });
    }
    return result;
}

}  // namespace decision
