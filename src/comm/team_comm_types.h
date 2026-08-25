// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 The Hong Kong University of Science and Technology (Guangzhou), Humanoid Computing & Learning Lab.

#pragma once

#include <cstdint>
#include <vector>

namespace comm {

/// Quantizable state sent by one player over the server speech channel.
struct TeamCommPacket {
    std::uint8_t version{1};
    std::uint8_t sender_player_number{0};
    double self_x_m{0.0};
    double self_y_m{0.0};
    bool fallen{false};
    bool ball_seen{false};
    double ball_x_m{0.0};
    double ball_y_m{0.0};
    bool opponent_seen{false};
    double opponent_x_m{0.0};
    double opponent_y_m{0.0};
    std::int8_t current_role{-1};
};

/// A decoded teammate packet annotated with its receive cycle.
struct TeamCommRecord {
    int sender_player_number{0};
    int server_cycle{0};
    double self_x_m{0.0};
    double self_y_m{0.0};
    bool fallen{false};
    bool ball_seen{false};
    double ball_x_m{0.0};
    double ball_y_m{0.0};
    bool opponent_seen{false};
    double opponent_x_m{0.0};
    double opponent_y_m{0.0};
    int current_role{-1};
};

/// Recent team records exposed to the world model as one cycle-level view.
struct TeamCommSnapshot {
    std::vector<TeamCommRecord> records;
};

}  // namespace comm
