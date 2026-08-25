// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 The Hong Kong University of Science and Technology (Guangzhou), Humanoid Computing & Learning Lab.

#pragma once

#include "src/world/frame_normalizer.h"

#include <optional>
#include <utility>
#include <string>
#include <unordered_map>
#include <vector>

namespace robot {

/// T1 joint naming, gains, symmetry, and head-camera geometry.
class T1RobotModel {
public:
    /// Clamped head yaw and pitch in degrees.
    struct HeadTargets {
        double yaw_deg{0.0};
        double pitch_deg{0.0};
    };

    T1RobotModel();

    const std::vector<std::string>& readable_joint_names() const;
    const std::vector<std::string>& actuator_names() const;

    const std::string& actuator_name_for_joint(const std::string& readable_joint_name) const;
    double joint_kp(const std::string& readable_joint_name) const;
    double joint_kd(const std::string& readable_joint_name) const;

    /// Motor names that share one logical joint under symmetry.
    struct SymmetryGroup {
        std::vector<std::string> motor_names;
        bool invert_direction{false};
    };
    const SymmetryGroup& symmetry_group(const std::string& logical_joint_name) const;

    /// Clamps head targets to the T1 joint limits.
    HeadTargets clamp_head_targets(double yaw_deg, double pitch_deg) const;
    /// Returns camera position in the torso frame, in meters.
    world::Vec3 camera_position_torso(double head_yaw_deg, double head_pitch_deg) const;
    std::optional<std::size_t> joint_order_index(const std::string& readable_joint_name) const;

private:
    struct GainPair {
        double kp{0.0};
        double kd{0.0};
    };

    std::vector<std::string> readable_joint_names_;
    std::vector<std::string> actuator_names_;
    std::unordered_map<std::string, std::string> readable_to_actuator_;
    std::unordered_map<std::string, GainPair> joint_gains_;
    std::unordered_map<std::string, SymmetryGroup> symmetry_groups_;
    std::unordered_map<std::string, std::size_t> joint_order_;
};

}  // namespace robot
