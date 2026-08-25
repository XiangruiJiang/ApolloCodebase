// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 The Hong Kong University of Science and Technology (Guangzhou), Humanoid Computing & Learning Lab.

#pragma once

#include "src/decision/high_level_command.h"
#include "src/behavior/onnx_session.h"
#include "src/robot/joint_targets.h"
#include "src/robot/t1_robot_model.h"
#include "src/world/world_snapshot.h"

#include <array>
#include <filesystem>
#include <optional>
#include <vector>

namespace behavior {

/// Policy inputs, outputs, and decoded targets from one walking step.
struct WalkStepResult {
    std::vector<float> observation;
    std::vector<float> action;
    robot::JointTargets joint_targets;
};

/// Executes the learned walking policy and its observation history.
class WalkRunner {
public:
    struct HeadTrackerState {
        std::optional<std::array<double, 2>> last_head_target_deg;
        std::optional<double> last_ball_seen_time;
    };

    explicit WalkRunner(const std::filesystem::path& model_path);

    /// Evaluates one policy step; `reset` reinitializes temporal observations.
    WalkStepResult step(
        const world::WorldSnapshot& snapshot,
        const decision::WalkCommand& command,
        bool reset,
        std::optional<int> role_id = std::nullopt);

private:
    static constexpr float kActionScale = 0.25F;
    static constexpr float kOrientationToAngVelScale = 0.2F;

    OnnxSession session_;
    robot::T1RobotModel robot_model_;
    std::vector<float> previous_action_;
    std::vector<float> observation_;
    std::vector<float> step_obs_buffer_;
    int history_length_{1};
    int step_obs_dim_{0};
    HeadTrackerState head_tracker_state_;

    std::array<float, 3> compute_velocity_command(
        const world::WorldSnapshot& snapshot,
        const decision::WalkCommand& command) const;
    void build_observation(
        const world::WorldSnapshot& snapshot,
        const std::array<float, 3>& velocity_command);
    robot::JointTargets decode_action(
        const world::WorldSnapshot& snapshot,
        const std::vector<float>& action,
        std::optional<int> role_id);
};

}  // namespace behavior
