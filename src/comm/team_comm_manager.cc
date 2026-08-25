// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 The Hong Kong University of Science and Technology (Guangzhou), Humanoid Computing & Learning Lab.

#include "src/comm/team_comm_manager.h"

#include <cmath>
#include <limits>

namespace comm {

namespace {

const world::PlayerObservation* select_shared_opponent(const world::WorldSnapshot& snapshot) {
    const world::PlayerObservation* best = nullptr;
    double best_dist = std::numeric_limits<double>::infinity();
    const std::array<double, 2> reference = snapshot.ball.visible
        ? std::array<double, 2>{snapshot.ball.position_m[0], snapshot.ball.position_m[1]}
        : std::array<double, 2>{snapshot.self.position_m[0], snapshot.self.position_m[1]};

    for (const auto& opponent : snapshot.opponents) {
        if (!opponent.seen) {
            continue;
        }
        const double dx = opponent.position_m[0] - reference[0];
        const double dy = opponent.position_m[1] - reference[1];
        const double dist = std::sqrt(dx * dx + dy * dy);
        if (dist < best_dist) {
            best = &opponent;
            best_dist = dist;
        }
    }

    return best;
}

}  // namespace

TeamCommManager::TeamCommManager(const std::string& team_name)
    : version_byte_(static_cast<std::uint8_t>((TeamCommCodec::team_hash(team_name) << 4) | kProtocolVersion)) {}

bool TeamCommManager::is_send_slot(int player_number, int server_cycle) const {
    return player_number > 0 && player_number <= 7 && (server_cycle % 14) == (2 * (player_number - 1));
}

TeamCommPacket TeamCommManager::make_packet(
    const world::WorldSnapshot& snapshot,
    int current_role) const {
    TeamCommPacket packet;
    packet.version = version_byte_;
    packet.sender_player_number = static_cast<std::uint8_t>(snapshot.player_number);
    packet.self_x_m = snapshot.self.position_m[0];
    packet.self_y_m = snapshot.self.position_m[1];
    packet.fallen = snapshot.self.position_m[2] < world::kFallenHeightThresholdM;
    packet.ball_seen = snapshot.ball.visible;
    packet.ball_x_m = snapshot.ball.position_m[0];
    packet.ball_y_m = snapshot.ball.position_m[1];
    if (const auto* shared_opponent = select_shared_opponent(snapshot); shared_opponent != nullptr) {
        packet.opponent_seen = true;
        packet.opponent_x_m = shared_opponent->position_m[0];
        packet.opponent_y_m = shared_opponent->position_m[1];
    }
    packet.current_role = static_cast<std::int8_t>(current_role);
    return packet;
}

void TeamCommManager::ingest(const TeamCommPacket& packet, int current_server_cycle) {
    // Reject packets from other teams (different version byte).
    if (packet.version != version_byte_) return;
    auto record = TeamCommCodec::to_record(packet);
    record.server_cycle = current_server_cycle;
    records_[packet.sender_player_number] = record;
}

TeamCommSnapshot TeamCommManager::make_snapshot(int current_server_cycle) const {
    TeamCommSnapshot snapshot;
    for (const auto& [_, record] : records_) {
        if (current_server_cycle - record.server_cycle <= kMaxRecordAgeCycles) {
            snapshot.records.push_back(record);
        }
    }
    return snapshot;
}

}  // namespace comm
