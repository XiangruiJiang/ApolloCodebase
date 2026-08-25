// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 The Hong Kong University of Science and Technology (Guangzhou), Humanoid Computing & Learning Lab.

#pragma once

#include "src/app/runtime_config.h"
#include "src/comm/team_comm_manager.h"
#include "src/decision/decision_manager.h"
#include "src/server/action_encoder.h"
#include "src/server/perception_parser.h"
#include "src/server/tcp_lpm_client.h"
#include "src/behavior/motion_manager.h"
#include "src/robot/t1_robot_model.h"
#include "src/world/world_state.h"

#include <cstddef>
#include <memory>
#include <string>

namespace app {

/// Owns the complete perception-to-action loop for one soccer agent.
class AgentApp {
public:
    explicit AgentApp(RuntimeConfig config);
    AgentApp(RuntimeConfig config, std::unique_ptr<server::TcpLpmClient> client);

    /// Runs until shutdown or a transport error terminates the agent.
    int run();
    /// Runs a bounded number of cycles, primarily for integration tests.
    int run_for_cycles(std::size_t cycles);
    void shutdown();
    /// Processes one server frame and returns the encoded action message.
    std::string process_perception_message(const std::string& message);

    const RuntimeConfig& config() const;
    bool shutdown_requested() const;
    const std::string& last_active_motion() const;

private:
    RuntimeConfig config_;
    bool shutdown_requested_{false};
    bool init_sent_{false};
    robot::T1RobotModel robot_model_;
    world::WorldState world_state_;
    server::PerceptionParser perception_parser_;
    decision::DecisionManager decision_manager_;
    behavior::MotionManager motion_manager_;
    comm::TeamCommManager team_comm_manager_;
    std::unique_ptr<server::TcpLpmClient> client_;
    std::size_t last_command_variant_index_{static_cast<std::size_t>(-1)};
    std::string last_active_motion_{"Neutral"};
};

}  // namespace app
