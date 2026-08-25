// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 The Hong Kong University of Science and Technology (Guangzhou), Humanoid Computing & Learning Lab.

#include "src/app/agent_app.h"
#include "src/app/runtime_config.h"

#include <exception>
#include <iostream>

int main(int argc, char* argv[]) {
    try {
        app::RuntimeConfig config = app::RuntimeConfig::from_args(argc, argv);
        app::AgentApp agent_app{config};
        return agent_app.run();
    } catch (const std::exception& ex) {
        std::cerr << "Failed to start agent: " << ex.what() << std::endl;
        return 1;
    }
}
