// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 The Hong Kong University of Science and Technology (Guangzhou), Humanoid Computing & Learning Lab.

#pragma once

#include <filesystem>
#include <string>

namespace app {

/// Command-line and runtime paths for a single agent process.
struct RuntimeConfig {
    std::string team_name{"Apollo3Drelease"};
    int player_number{1};
    std::string host{"127.0.0.1"};
    int port{60000};
    std::string asset_root{"assets"};
    std::string config_root{"config"};
    std::string log_level{"info"};

    /// Parses supported command-line options and preserves unspecified defaults.
    static RuntimeConfig from_args(int argc, char* argv[]);

    /// Resolves an asset path against the configured or project asset root.
    std::filesystem::path resolve_asset_path(const std::string& relative_asset_path) const {
        const std::filesystem::path root(asset_root);
        if (root.is_absolute()) {
            return root / relative_asset_path;
        }
#ifdef APOLLO_CODE_BASE_PROJECT_SOURCE_DIR
        return std::filesystem::path(APOLLO_CODE_BASE_PROJECT_SOURCE_DIR) / root / relative_asset_path;
#else
        return std::filesystem::current_path() / root / relative_asset_path;
#endif
    }
};

}  // namespace app
