// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 The Hong Kong University of Science and Technology (Guangzhou), Humanoid Computing & Learning Lab.

#pragma once

#include "src/server/perception_types.h"

#include <string>

namespace server {

/// Parses one RCSSServerMJ perception message into typed protocol data.
class PerceptionParser {
public:
    /// Parses a complete frame or throws when a required value is malformed.
    PerceptionFrame parse(const std::string& message) const;
};

}  // namespace server
