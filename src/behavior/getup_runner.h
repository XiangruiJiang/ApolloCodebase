// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 The Hong Kong University of Science and Technology (Guangzhou), Humanoid Computing & Learning Lab.

#pragma once

#include "src/behavior/onnx_session.h"
#include "src/robot/joint_targets.h"
#include "src/robot/t1_robot_model.h"
#include "src/world/world_snapshot.h"

#include <filesystem>
#include <vector>

namespace behavior {

/// Joint targets and upright status produced by one recovery-policy step.
struct GetupStepResult {
    robot::JointTargets joint_targets;
    bool upright{false};
};

/// Executes the learned get-up policy and retains its recurrent action history.
class GetupRunner {
public:
    explicit GetupRunner(const std::filesystem::path& model_path);

    /// Evaluates one policy step; `reset` starts a new recovery sequence.
    GetupStepResult step(
        const world::WorldSnapshot& snapshot,
        bool reset);

private:
    static constexpr float kActionScale = 0.6F;
    static constexpr float kActionClip = 5.0F;
    // gravity_body.z threshold for "upright" detection.
    // Standing: gravity_body ~ (0,0,-1) -> z ~ -1.0
    // Threshold -0.85 corresponds to ~32 degrees from vertical.
    static constexpr double kUprightGravityZThreshold = -0.85;

    OnnxSession session_;
    robot::T1RobotModel robot_model_;
    std::vector<float> previous_action_;
    std::vector<float> observation_buffer_;

    void build_observation(
        const world::WorldSnapshot& snapshot);
    robot::JointTargets decode_action(
        const world::WorldSnapshot& snapshot,
        const std::vector<float>& action);
};

}  // namespace behavior
