// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 The Hong Kong University of Science and Technology (Guangzhou), Humanoid Computing & Learning Lab.

#include "src/decision/blackboard.h"

namespace decision {

bool Blackboard::exists(const std::string& key) const {
    return values_.find(key) != values_.end();
}

void Blackboard::clear() {
    values_.clear();
}

}  // namespace decision
