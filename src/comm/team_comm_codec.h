// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 The Hong Kong University of Science and Technology (Guangzhou), Humanoid Computing & Learning Lab.

#pragma once

#include "src/comm/team_comm_types.h"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace comm {

/// Encodes the fixed-size, lossy team communication packet format.
class TeamCommCodec {
public:
    static constexpr std::size_t kPacketSizeBytes = 8U;

    /// Computes the 4-bit team hash stored in the version byte's high nibble.
    static std::uint8_t team_hash(const std::string& team_name);

    static std::vector<std::uint8_t> encode(const TeamCommPacket& packet);
    static TeamCommPacket decode(const std::vector<std::uint8_t>& encoded);
    static TeamCommRecord to_record(const TeamCommPacket& packet);
};

}  // namespace comm
