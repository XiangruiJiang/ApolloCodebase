// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 The Hong Kong University of Science and Technology (Guangzhou), Humanoid Computing & Learning Lab.

#pragma once

#include <any>
#include <string>
#include <unordered_map>

namespace decision {

/// Per-cycle typed storage shared by decision tasks.
class Blackboard {
public:
    // Blackboard keys — the get/set/exists APIs are stringly typed, so use
    // these constants at every site to avoid silent fallbacks when a typo
    // drifts one side off the other.
    static constexpr const char* kKeyCurrentRole = "current_role";
    static constexpr const char* kKeyRolePos = "role_pos";

    template <typename T>
    void set(const std::string& key, T value) {
        values_[key] = std::move(value);
    }

    template <typename T>
    const T& get(const std::string& key) const {
        return std::any_cast<const T&>(values_.at(key));
    }

    bool exists(const std::string& key) const;
    void clear();

private:
    std::unordered_map<std::string, std::any> values_;
};

}  // namespace decision
