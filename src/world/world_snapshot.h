// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 The Hong Kong University of Science and Technology (Guangzhou), Humanoid Computing & Learning Lab.

#pragma once

#include "src/comm/team_comm_types.h"
#include "src/world/frame_normalizer.h"
#include "src/world/play_mode.h"

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace world {

constexpr double kFallenHeightThresholdM = 0.3;


/// Estimated state of the controlled robot in the canonical team frame.
struct SelfState {
    Vec3 position_m{0.0, 0.0, 0.0};
    QuaternionWxyz orientation_wxyz{1.0, 0.0, 0.0, 0.0};
    Vec3 gyro_deg_s{0.0, 0.0, 0.0};
    Vec3 accel_mps2{0.0, 0.0, 0.0};
    Vec3 lin_vel_b{0.0, 0.0, 0.0};
    std::unordered_map<std::string, double> joint_positions_deg;
    std::unordered_map<std::string, double> joint_velocities_deg_s;
};

/// Best available ball estimate in the canonical team frame.
struct BallState {
    bool visible{false};
    Vec3 position_m{0.0, 0.0, 0.0};
    Vec3 velocity_mps{0.0, 0.0, 0.0};
    bool velocity_valid{false};
};

/// Local or team-shared estimate of one player.
struct PlayerObservation {
    int player_number{0};
    bool is_teammate{false};
    bool seen{false};
    double last_seen_time{-1.0};
    bool fallen{false};
    Vec3 position_m{0.0, 0.0, 0.0};
    int comm_role{-1};  // role reported via team comm, -1 if unknown
};

/// Read-only decision input assembled for the current server cycle.
struct WorldSnapshot {
    std::string team_name;
    int player_number{0};
    std::optional<bool> is_left_team;
    double server_time{0.0};
    double last_server_time{0.0};
    PlayMode play_mode{PlayMode::NotInitialized};
    PlayModeGroup play_mode_group{PlayModeGroup::NotInitialized};
    bool has_beamed{false};
    SelfState self;
    BallState ball;
    std::vector<PlayerObservation> teammates;
    std::vector<PlayerObservation> opponents;
    std::vector<PlayerObservation> shared_opponents;
    comm::TeamCommSnapshot team_comm_snapshot;
};

}  // namespace world
