// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 The Hong Kong University of Science and Technology (Guangzhou), Humanoid Computing & Learning Lab.

#pragma once

#include "src/behavior/onnx_session.h"
#include "src/robot/joint_targets.h"
#include "src/robot/t1_robot_model.h"

#include <filesystem>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace behavior {

/// Joint targets and completion status produced by one keyframe step.
struct KeyframeStepResult {
    robot::JointTargets joint_targets;
    bool finished{false};
};

/// Loads and interpolates a YAML keyframe motion.
class KeyframeRunner {
public:
    explicit KeyframeRunner(const std::filesystem::path& yaml_path);

    /// Advances the motion at server time; `reset` restarts from its first frame.
    KeyframeStepResult step(bool reset, double server_time);

private:
    struct Keyframe {
        double delta{0.0};
        std::unordered_map<std::string, double> motor_positions;
        std::optional<double> kp;
        std::optional<double> kd;
        std::unordered_map<std::string, double> p_gains;
        std::unordered_map<std::string, double> d_gains;
    };

    robot::T1RobotModel robot_model_;
    bool has_symmetry_{false};
    double default_kp_{0.0};
    double default_kd_{0.0};
    std::vector<Keyframe> keyframes_;
    std::size_t keyframe_step_{0};
    double keyframe_start_time_{0.0};
};

}  // namespace behavior
