// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 The Hong Kong University of Science and Technology (Guangzhou), Humanoid Computing & Learning Lab.

#include "src/comm/team_comm_codec.h"

#include "src/server/server_constants.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace comm {

namespace {

constexpr std::uint8_t kInvalidQuantized = 0xFFU;
constexpr std::uint8_t kUnknownRoleBits = 0x07U;

using server_constants::kFieldHalfLengthM;
using server_constants::kFieldHalfWidthM;

std::uint8_t quantize_axis(double value, double min_value, double max_value) {
    const double clamped = std::clamp(value, min_value, max_value);
    const double normalized = (clamped - min_value) / (max_value - min_value);
    return static_cast<std::uint8_t>(std::lround(normalized * 254.0));
}

double dequantize_axis(std::uint8_t value, double min_value, double max_value) {
    if (value == kInvalidQuantized) {
        return 0.0;
    }
    const double normalized = static_cast<double>(value) / 254.0;
    return min_value + normalized * (max_value - min_value);
}

std::uint8_t pack_header(const TeamCommPacket& packet) {
    const std::uint8_t sender_bits = static_cast<std::uint8_t>(
        std::clamp<int>(static_cast<int>(packet.sender_player_number), 1, 7) - 1);
    const std::uint8_t role_bits = packet.current_role >= 0 && packet.current_role <= 6
        ? static_cast<std::uint8_t>(packet.current_role)
        : kUnknownRoleBits;
    return static_cast<std::uint8_t>(
        sender_bits |
        static_cast<std::uint8_t>(role_bits << 3U) |
        static_cast<std::uint8_t>(packet.fallen ? 0x40U : 0x00U));
}

}  // namespace

std::uint8_t TeamCommCodec::team_hash(const std::string& team_name) {
    // DJB2 hash truncated to 4 bits. Different team names almost always
    // produce different nibbles, filtering out opponent messages.
    std::uint32_t h = 5381U;
    for (char c : team_name) {
        h = ((h << 5) + h) + static_cast<std::uint8_t>(c);
    }
    return static_cast<std::uint8_t>(h & 0x0FU);
}

std::vector<std::uint8_t> TeamCommCodec::encode(const TeamCommPacket& packet) {
    std::vector<std::uint8_t> encoded;
    encoded.reserve(kPacketSizeBytes);
    encoded.push_back(packet.version);
    encoded.push_back(pack_header(packet));
    encoded.push_back(quantize_axis(packet.self_x_m, -kFieldHalfLengthM, kFieldHalfLengthM));
    encoded.push_back(quantize_axis(packet.self_y_m, -kFieldHalfWidthM, kFieldHalfWidthM));
    encoded.push_back(packet.ball_seen
        ? quantize_axis(packet.ball_x_m, -kFieldHalfLengthM, kFieldHalfLengthM)
        : kInvalidQuantized);
    encoded.push_back(packet.ball_seen
        ? quantize_axis(packet.ball_y_m, -kFieldHalfWidthM, kFieldHalfWidthM)
        : kInvalidQuantized);
    encoded.push_back(packet.opponent_seen
        ? quantize_axis(packet.opponent_x_m, -kFieldHalfLengthM, kFieldHalfLengthM)
        : kInvalidQuantized);
    encoded.push_back(packet.opponent_seen
        ? quantize_axis(packet.opponent_y_m, -kFieldHalfWidthM, kFieldHalfWidthM)
        : kInvalidQuantized);
    return encoded;
}

TeamCommPacket TeamCommCodec::decode(const std::vector<std::uint8_t>& encoded) {
    if (encoded.size() != kPacketSizeBytes) {
        throw std::runtime_error("Team communication packet size mismatch");
    }
    TeamCommPacket packet;
    packet.version = encoded[0];
    const std::uint8_t header = encoded[1];
    packet.sender_player_number = static_cast<std::uint8_t>((header & 0x07U) + 1U);
    const std::uint8_t role_bits = static_cast<std::uint8_t>((header >> 3U) & 0x07U);
    packet.current_role = role_bits == kUnknownRoleBits
        ? static_cast<std::int8_t>(-1)
        : static_cast<std::int8_t>(role_bits);
    packet.fallen = (header & 0x40U) != 0U;
    packet.self_x_m = dequantize_axis(encoded[2], -kFieldHalfLengthM, kFieldHalfLengthM);
    packet.self_y_m = dequantize_axis(encoded[3], -kFieldHalfWidthM, kFieldHalfWidthM);
    packet.ball_seen = encoded[4] != kInvalidQuantized && encoded[5] != kInvalidQuantized;
    if (packet.ball_seen) {
        packet.ball_x_m = dequantize_axis(encoded[4], -kFieldHalfLengthM, kFieldHalfLengthM);
        packet.ball_y_m = dequantize_axis(encoded[5], -kFieldHalfWidthM, kFieldHalfWidthM);
    }
    packet.opponent_seen = encoded[6] != kInvalidQuantized && encoded[7] != kInvalidQuantized;
    if (packet.opponent_seen) {
        packet.opponent_x_m = dequantize_axis(encoded[6], -kFieldHalfLengthM, kFieldHalfLengthM);
        packet.opponent_y_m = dequantize_axis(encoded[7], -kFieldHalfWidthM, kFieldHalfWidthM);
    }
    return packet;
}

TeamCommRecord TeamCommCodec::to_record(const TeamCommPacket& packet) {
    return {
        static_cast<int>(packet.sender_player_number),
        0,
        packet.self_x_m,
        packet.self_y_m,
        packet.fallen,
        packet.ball_seen,
        packet.ball_x_m,
        packet.ball_y_m,
        packet.opponent_seen,
        packet.opponent_x_m,
        packet.opponent_y_m,
        static_cast<int>(packet.current_role),
    };
}

}  // namespace comm
