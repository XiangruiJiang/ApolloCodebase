// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 The Hong Kong University of Science and Technology (Guangzhou), Humanoid Computing & Learning Lab.

#pragma once

#include "src/comm/team_comm_types.h"
#include "src/server/perception_types.h"
#include "src/robot/t1_robot_model.h"
#include "src/world/ball_kalman.h"
#include "src/world/world_snapshot.h"

#include <optional>
#include <string>
#include <vector>

namespace world {

/// Fuses raw perception and team communication into a canonical world snapshot.
class WorldState {
public:
    WorldState(std::string team_name, int player_number, int max_players_per_team);

    /// Applies one perception frame and refreshes all derived estimates.
    void update_from_perception(
        const server::PerceptionFrame& frame,
        const robot::T1RobotModel& robot_model,
        bool normalized_is_left_team);

    const WorldSnapshot& snapshot() const { return snapshot_; }

    void set_has_beamed(bool value) { snapshot_.has_beamed = value; }
    void set_team_comm_snapshot(const comm::TeamCommSnapshot& comm_snapshot);

private:
    WorldSnapshot snapshot_;
    Vec3 last_known_ball_position_m_{0.0, 0.0, 0.0};
    double last_known_ball_time_{-1.0};
    BallKalman ball_kalman_;

    // Per-opponent motion-gate + smoothing state (indexed by player_number - 1).
    // Opponents get no teleport gate and no temporal filter otherwise, so a
    // false-positive / misclassified detection can snap a slot across the field.
    std::vector<Vec3> last_opponent_position_m_;
    std::vector<double> last_opponent_time_;

    enum class CornerProbeState {
        Idle,
        Probing,
        Locked,
    };
    CornerProbeState corner_probe_state_ = CornerProbeState::Idle;
    Vec3 corner_anchor_position_m_{0.0, 0.0, 0.0};
    double corner_probe_start_time_{-1.0};

    static std::string normalize_joint_name(const std::string& name);
};

}  // namespace world
