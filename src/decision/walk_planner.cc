// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 The Hong Kong University of Science and Technology (Guangzhou), Humanoid Computing & Learning Lab.

#include "src/decision/walk_planner.h"

#include "src/decision/field_geometry.h"
#include "src/decision/opponent_view.h"
#include "src/decision/role_manager.h"
#include "src/math/math_utils.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <queue>
#include <vector>

namespace decision {

namespace {

using field_geometry::Position2;

// Maximum obstacles: 7 opponents + 6 teammates (excluding self).
constexpr int kMaxObstacles = 13;

constexpr double kAStarResolutionM = 0.5;
constexpr double kAStarStartRelaxM = 0.8;
constexpr double kAStarLineSafetyMarginM = 0.05;
constexpr double kAStarFieldSafetyMarginM = 0.15;
constexpr double kAStarForwardGoalExtensionM = 1.2;
constexpr double kAStarUnboundedPaddingM = 2.0;
constexpr double kBoundarySoftCost = 1.5;

constexpr double kWalkHardObstacleRadiusM = 0.75;
constexpr double kWalkSoftObstacleRadiusM = 1.8;
constexpr double kWalkObstacleCost = 3.0;
constexpr double kWalkProtectedApHardObstacleRadiusM = 1.15;
constexpr double kWalkProtectedApSoftObstacleRadiusM = 2.8;
constexpr double kWalkProtectedApObstacleCost = 7.0;
constexpr double kWalkLookaheadM = 2.0;

constexpr double kHalfLengthM = field_geometry::kActualHalfLengthM;
constexpr double kHalfWidthM = field_geometry::kActualHalfWidthM;

// Local alias for the planner's boundary-cost trigger so the math reads cleanly
// at every site (vs. the fully-qualified `field_geometry::kWalkPlannerFieldMarginM`).
constexpr double kFieldMarginM = field_geometry::kWalkPlannerFieldMarginM;

struct Obstacle {
    Position2 position{0.0, 0.0};
    bool protects_ap{false};
};

struct ObstacleBuffer {
    std::array<Obstacle, kMaxObstacles> data;
    int count{0};
};

struct PlannerConfig {
    double hard_obstacle_radius_m{0.0};
    double soft_obstacle_radius_m{0.0};
    double obstacle_cost{0.0};
    double protected_ap_hard_obstacle_radius_m{0.0};
    double protected_ap_soft_obstacle_radius_m{0.0};
    double protected_ap_obstacle_cost{0.0};
    double lookahead_m{kWalkLookaheadM};
    bool avoid_field_boundaries{true};
    bool disable_forward_boundary{false};
};

struct PlanningBounds {
    double min_x{0.0};
    double max_x{0.0};
    double min_y{0.0};
    double max_y{0.0};
};

struct AStarGrid {
    PlanningBounds bounds;
    int cols{0};
    int rows{0};

    int index(int x, int y) const {
        return y * cols + x;
    }

    bool contains_cell(int x, int y) const {
        return x >= 0 && x < cols && y >= 0 && y < rows;
    }

    int cell_x(double x_m) const {
        const int x = static_cast<int>(std::round((x_m - bounds.min_x) / kAStarResolutionM));
        return std::clamp(x, 0, cols - 1);
    }

    int cell_y(double y_m) const {
        const int y = static_cast<int>(std::round((y_m - bounds.min_y) / kAStarResolutionM));
        return std::clamp(y, 0, rows - 1);
    }

    int nearest_index(const Position2& position) const {
        return index(cell_x(position[0]), cell_y(position[1]));
    }

    Position2 position(int idx) const {
        const int x = idx % cols;
        const int y = idx / cols;
        return {
            bounds.min_x + static_cast<double>(x) * kAStarResolutionM,
            bounds.min_y + static_cast<double>(y) * kAStarResolutionM,
        };
    }
};

struct SearchNode {
    double g{std::numeric_limits<double>::infinity()};
    int parent{-1};
    bool closed{false};
};

struct QueueEntry {
    int index{0};
    double f{0.0};
    double g{0.0};
};

struct QueueEntryGreater {
    bool operator()(const QueueEntry& lhs, const QueueEntry& rhs) const {
        return lhs.f > rhs.f;
    }
};

void collect_obstacles(
    ObstacleBuffer& out,
    const world::WorldSnapshot& snapshot,
    int self_player_number,
    bool include_opponents,
    bool all_teammates,
    std::optional<double> opponent_x_threshold = std::nullopt) {
    out.count = 0;

    if (include_opponents) {
        for (const auto& opp : collect_known_opponent_positions(snapshot)) {
            if (out.count >= kMaxObstacles) break;
            if (opponent_x_threshold.has_value() && opp[0] >= opponent_x_threshold.value()) {
                continue;
            }
            out.data[out.count++] = {opp, false};
        }
    }

    for (const auto& mate : snapshot.teammates) {
        if (out.count >= kMaxObstacles) break;
        if (mate.player_number == self_player_number) continue;
        if (!all_teammates && !mate.fallen) continue;
        if (!mate.seen && (snapshot.server_time - mate.last_seen_time > 2.0)) continue;
        out.data[out.count++] = {
            {mate.position_m[0], mate.position_m[1]},
            mate.comm_role == RoleManager::ROLE_AP,
        };
    }
}

double heading_to(const Position2& origin, const Position2& target) {
    return math::vector_angle_deg({target[0] - origin[0], target[1] - origin[1]});
}

WalkPlan make_direct_plan(
    const Position2& origin,
    const Position2& target) {
    return {heading_to(origin, target)};
}

PlanningBounds make_planning_bounds(
    const Position2& origin,
    const Position2& target,
    const PlannerConfig& config) {
    PlanningBounds bounds;
    if (config.avoid_field_boundaries) {
        bounds = {
            -kHalfLengthM + kAStarFieldSafetyMarginM,
            kHalfLengthM - kAStarFieldSafetyMarginM,
            -kHalfWidthM + kAStarFieldSafetyMarginM,
            kHalfWidthM - kAStarFieldSafetyMarginM,
        };
        if (config.disable_forward_boundary) {
            bounds.max_x = kHalfLengthM + kAStarForwardGoalExtensionM;
        }
    } else {
        bounds = {
            -kHalfLengthM - kAStarUnboundedPaddingM,
            kHalfLengthM + kAStarUnboundedPaddingM,
            -kHalfWidthM - kAStarUnboundedPaddingM,
            kHalfWidthM + kAStarUnboundedPaddingM,
        };
        bounds.min_x = std::min(bounds.min_x, std::min(origin[0], target[0]) - kAStarUnboundedPaddingM);
        bounds.max_x = std::max(bounds.max_x, std::max(origin[0], target[0]) + kAStarUnboundedPaddingM);
        bounds.min_y = std::min(bounds.min_y, std::min(origin[1], target[1]) - kAStarUnboundedPaddingM);
        bounds.max_y = std::max(bounds.max_y, std::max(origin[1], target[1]) + kAStarUnboundedPaddingM);
    }
    return bounds;
}

bool position_in_bounds(const Position2& position, const PlanningBounds& bounds) {
    constexpr double kEpsilon = 1e-6;
    return position[0] >= bounds.min_x - kEpsilon &&
           position[0] <= bounds.max_x + kEpsilon &&
           position[1] >= bounds.min_y - kEpsilon &&
           position[1] <= bounds.max_y + kEpsilon;
}

Position2 clamp_to_bounds(const Position2& position, const PlanningBounds& bounds) {
    return {
        std::clamp(position[0], bounds.min_x, bounds.max_x),
        std::clamp(position[1], bounds.min_y, bounds.max_y),
    };
}

AStarGrid make_grid(const PlanningBounds& bounds) {
    const double span_x = std::max(kAStarResolutionM, bounds.max_x - bounds.min_x);
    const double span_y = std::max(kAStarResolutionM, bounds.max_y - bounds.min_y);
    return {
        bounds,
        static_cast<int>(std::ceil(span_x / kAStarResolutionM)) + 1,
        static_cast<int>(std::ceil(span_y / kAStarResolutionM)) + 1,
    };
}

bool in_start_relaxation(const Position2& position, const Position2& start) {
    return math::planar_dist(position, start) <= kAStarStartRelaxM;
}

double boundary_traversal_cost(const Position2& position, const PlannerConfig& config) {
    if (!config.avoid_field_boundaries) {
        return 0.0;
    }

    double cost = 0.0;
    auto add_boundary = [&](double clearance_m) {
        if (clearance_m <= 0.0) {
            return std::numeric_limits<double>::infinity();
        }
        if (clearance_m >= kFieldMarginM) {
            return 0.0;
        }
        const double t = (kFieldMarginM - clearance_m) / kFieldMarginM;
        return kBoundarySoftCost * t * t;
    };

    const double left = add_boundary(position[0] + kHalfLengthM);
    const double top = add_boundary(kHalfWidthM - position[1]);
    const double bottom = add_boundary(position[1] + kHalfWidthM);
    if (!std::isfinite(left) || !std::isfinite(top) || !std::isfinite(bottom)) {
        return std::numeric_limits<double>::infinity();
    }
    cost += left + top + bottom;

    if (!config.disable_forward_boundary) {
        const double right = add_boundary(kHalfLengthM - position[0]);
        if (!std::isfinite(right)) {
            return std::numeric_limits<double>::infinity();
        }
        cost += right;
    }
    return cost;
}

bool near_planned_boundary(const Position2& position, const PlannerConfig& config) {
    if (!config.avoid_field_boundaries) {
        return false;
    }

    if (position[0] + kHalfLengthM < kFieldMarginM ||
        kHalfWidthM - position[1] < kFieldMarginM ||
        position[1] + kHalfWidthM < kFieldMarginM) {
        return true;
    }

    return !config.disable_forward_boundary &&
           kHalfLengthM - position[0] < kFieldMarginM;
}

double hard_radius_for_obstacle(const PlannerConfig& config, const Obstacle& obstacle) {
    return obstacle.protects_ap
        ? std::max(config.hard_obstacle_radius_m, config.protected_ap_hard_obstacle_radius_m)
        : config.hard_obstacle_radius_m;
}

double soft_radius_for_obstacle(
    const PlannerConfig& config,
    const Obstacle& obstacle,
    double hard_radius_m) {
    const double configured_soft = obstacle.protects_ap
        ? std::max(config.soft_obstacle_radius_m, config.protected_ap_soft_obstacle_radius_m)
        : config.soft_obstacle_radius_m;
    return std::max(configured_soft, hard_radius_m);
}

double cost_for_obstacle(const PlannerConfig& config, const Obstacle& obstacle) {
    return obstacle.protects_ap
        ? std::max(config.obstacle_cost, config.protected_ap_obstacle_cost)
        : config.obstacle_cost;
}

double obstacle_traversal_cost(
    const Position2& position,
    const ObstacleBuffer& obstacles,
    const PlannerConfig& config,
    const Position2& start) {
    if (config.hard_obstacle_radius_m <= 0.0 &&
        config.soft_obstacle_radius_m <= 0.0 &&
        config.protected_ap_hard_obstacle_radius_m <= 0.0 &&
        config.protected_ap_soft_obstacle_radius_m <= 0.0) {
        return 0.0;
    }

    const bool start_relaxed = in_start_relaxation(position, start);
    double cost = 0.0;
    for (int i = 0; i < obstacles.count; ++i) {
        const auto& obstacle = obstacles.data[i];
        const double hard_radius = hard_radius_for_obstacle(config, obstacle);
        const double soft_radius = soft_radius_for_obstacle(config, obstacle, hard_radius);
        const double cost_band = std::max(soft_radius - hard_radius, 1e-3);
        const double distance_m = math::planar_dist(position, obstacle.position);
        if (!start_relaxed && distance_m < hard_radius) {
            return std::numeric_limits<double>::infinity();
        }
        if (soft_radius > 0.0 && distance_m < soft_radius) {
            const double effective_distance =
                start_relaxed ? std::max(distance_m, hard_radius) : distance_m;
            const double t = std::clamp((soft_radius - effective_distance) / cost_band, 0.0, 1.0);
            cost += cost_for_obstacle(config, obstacle) * t * t;
        }
    }
    return cost;
}

double cell_traversal_cost(
    const Position2& position,
    const ObstacleBuffer& obstacles,
    const PlannerConfig& config,
    const Position2& start) {
    const double obstacle_cost =
        obstacle_traversal_cost(position, obstacles, config, start);
    if (!std::isfinite(obstacle_cost)) {
        return std::numeric_limits<double>::infinity();
    }

    const double boundary_cost = boundary_traversal_cost(position, config);
    if (!std::isfinite(boundary_cost)) {
        return std::numeric_limits<double>::infinity();
    }

    return 1.0 + obstacle_cost + boundary_cost;
}

bool line_segment_safe(
    const Position2& start,
    const Position2& end,
    const ObstacleBuffer& obstacles,
    const PlannerConfig& config) {
    if (config.hard_obstacle_radius_m <= 0.0 &&
        config.protected_ap_hard_obstacle_radius_m <= 0.0) {
        return true;
    }

    for (int i = 0; i < obstacles.count; ++i) {
        const double hard_radius = hard_radius_for_obstacle(config, obstacles.data[i]);
        const double safe_distance = hard_radius + kAStarLineSafetyMarginM;
        if (math::point_segment_distance(obstacles.data[i].position, start, end) < safe_distance) {
            return false;
        }
    }
    return true;
}

double heuristic(const Position2& position, const Position2& goal) {
    return math::planar_dist(position, goal);
}

std::vector<Position2> reconstruct_path(
    const AStarGrid& grid,
    const std::vector<SearchNode>& nodes,
    int start_idx,
    int goal_idx) {
    std::vector<Position2> path;
    for (int cursor = goal_idx; cursor >= 0; cursor = nodes[cursor].parent) {
        path.push_back(grid.position(cursor));
        if (cursor == start_idx) {
            break;
        }
    }
    std::reverse(path.begin(), path.end());
    return path;
}

std::optional<std::vector<Position2>> find_astar_path(
    const Position2& origin,
    const Position2& target,
    const ObstacleBuffer& obstacles,
    const PlannerConfig& config) {
    const PlanningBounds bounds = make_planning_bounds(origin, target, config);
    const AStarGrid grid = make_grid(bounds);

    const Position2 start = clamp_to_bounds(origin, bounds);
    const Position2 goal = clamp_to_bounds(target, bounds);
    const int start_idx = grid.nearest_index(start);
    const int goal_idx = grid.nearest_index(goal);

    if (start_idx == goal_idx) {
        return std::vector<Position2>{start, goal};
    }

    const std::size_t node_count = static_cast<std::size_t>(grid.cols * grid.rows);
    std::vector<SearchNode> nodes(node_count);
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueEntryGreater> open;

    nodes[start_idx].g = 0.0;
    nodes[start_idx].parent = -1;
    open.push({start_idx, heuristic(start, goal), 0.0});

    int best_idx = start_idx;
    double best_h = heuristic(start, goal);

    constexpr std::array<int, 8> kDx{{1, 1, 0, -1, -1, -1, 0, 1}};
    constexpr std::array<int, 8> kDy{{0, 1, 1, 1, 0, -1, -1, -1}};

    while (!open.empty()) {
        const QueueEntry entry = open.top();
        open.pop();

        SearchNode& current = nodes[entry.index];
        if (current.closed || entry.g > current.g + 1e-9) {
            continue;
        }
        current.closed = true;

        const Position2 current_position = grid.position(entry.index);
        const double current_h = heuristic(current_position, goal);
        if (current_h < best_h) {
            best_h = current_h;
            best_idx = entry.index;
        }
        if (entry.index == goal_idx) {
            return reconstruct_path(grid, nodes, start_idx, goal_idx);
        }

        const int cx = entry.index % grid.cols;
        const int cy = entry.index / grid.cols;
        for (std::size_t i = 0; i < kDx.size(); ++i) {
            const int nx = cx + kDx[i];
            const int ny = cy + kDy[i];
            if (!grid.contains_cell(nx, ny)) {
                continue;
            }

            const int next_idx = grid.index(nx, ny);
            if (nodes[next_idx].closed) {
                continue;
            }

            const Position2 next_position = grid.position(next_idx);
            if (!position_in_bounds(next_position, bounds)) {
                continue;
            }

            const double traversal =
                cell_traversal_cost(next_position, obstacles, config, start);
            if (!std::isfinite(traversal)) {
                continue;
            }

            const double step_distance =
                std::hypot(static_cast<double>(kDx[i]), static_cast<double>(kDy[i])) *
                kAStarResolutionM;
            const double tentative_g = current.g + step_distance * traversal;
            if (tentative_g + 1e-9 >= nodes[next_idx].g) {
                continue;
            }

            nodes[next_idx].g = tentative_g;
            nodes[next_idx].parent = entry.index;
            open.push({
                next_idx,
                tentative_g + heuristic(next_position, goal),
                tentative_g,
            });
        }
    }

    const double initial_h = heuristic(start, goal);
    if (best_idx != start_idx && best_h < initial_h - kAStarResolutionM) {
        return reconstruct_path(grid, nodes, start_idx, best_idx);
    }
    return std::nullopt;
}

Position2 select_waypoint(
    const Position2& origin,
    const Position2& target,
    const std::vector<Position2>& path,
    const ObstacleBuffer& obstacles,
    const PlannerConfig& config) {
    if (path.size() < 2) {
        return target;
    }

    Position2 candidate = path[1];
    double travelled_m = 0.0;
    for (std::size_t i = 1; i < path.size(); ++i) {
        travelled_m += math::planar_dist(path[i - 1], path[i]);
        if (travelled_m > config.lookahead_m) {
            break;
        }

        if (!line_segment_safe(origin, path[i], obstacles, config)) {
            if (i > 1) {
                break;
            }
            continue;
        }
        candidate = path[i];
    }
    return candidate;
}

WalkPlan compute_astar_plan(
    const Position2& origin,
    const Position2& target,
    const ObstacleBuffer& obstacles,
    const PlannerConfig& config) {
    if (math::planar_dist(origin, target) < 1e-4) {
        return {0.0};
    }

    const bool obstacles_disabled =
        config.hard_obstacle_radius_m <= 0.0 && config.soft_obstacle_radius_m <= 0.0;
    const bool boundary_planning_needed =
        near_planned_boundary(origin, config) || near_planned_boundary(target, config);
    if (obstacles_disabled || (obstacles.count == 0 && !boundary_planning_needed)) {
        return make_direct_plan(origin, target);
    }

    const auto path = find_astar_path(origin, target, obstacles, config);
    if (!path.has_value()) {
        return make_direct_plan(origin, target);
    }

    const Position2 waypoint =
        select_waypoint(origin, target, path.value(), obstacles, config);
    return {heading_to(origin, waypoint)};
}

}  // namespace

WalkPlan plan_walk(
    const std::array<double, 2>& self_pos,
    const std::array<double, 2>& target_pos,
    const world::WorldSnapshot& snapshot,
    int self_player_number,
    std::optional<double> opponent_x_threshold,
    bool avoid_field_boundaries,
    bool avoid_obstacles) {

    const double dist = math::planar_dist(self_pos, target_pos);
    if (dist < field_geometry::kNearTargetM) {
        return {heading_to(self_pos, target_pos)};
    }

    ObstacleBuffer obstacles;
    if (avoid_obstacles) {
        collect_obstacles(obstacles, snapshot, self_player_number, true, true, opponent_x_threshold);
    }

    PlannerConfig config;
    config.hard_obstacle_radius_m = kWalkHardObstacleRadiusM;
    config.soft_obstacle_radius_m = kWalkSoftObstacleRadiusM;
    config.obstacle_cost = kWalkObstacleCost;
    config.protected_ap_hard_obstacle_radius_m = kWalkProtectedApHardObstacleRadiusM;
    config.protected_ap_soft_obstacle_radius_m = kWalkProtectedApSoftObstacleRadiusM;
    config.protected_ap_obstacle_cost = kWalkProtectedApObstacleCost;
    config.lookahead_m = kWalkLookaheadM;
    config.avoid_field_boundaries = avoid_field_boundaries;
    config.disable_forward_boundary = true;
    const auto plan = compute_astar_plan(self_pos, target_pos, obstacles, config);
    return {plan.heading_deg};
}

}  // namespace decision