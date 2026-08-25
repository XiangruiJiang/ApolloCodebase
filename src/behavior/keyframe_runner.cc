// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 The Hong Kong University of Science and Technology (Guangzhou), Humanoid Computing & Learning Lab.

#include "src/behavior/keyframe_runner.h"

#include <stdexcept>

namespace behavior {

KeyframeRunner::KeyframeRunner(const std::filesystem::path& yaml_path) {
    const YAML::Node root = OnnxSession::load_yaml(yaml_path);
    has_symmetry_ = root["symmetry"].as<bool>();
    default_kp_ = root["kp"].as<double>();
    default_kd_ = root["kd"].as<double>();

    for (const auto& keyframe_node : root["keyframes"]) {
        Keyframe keyframe;
        keyframe.delta = keyframe_node["delta"].as<double>();
        if (keyframe_node["kp"]) {
            keyframe.kp = keyframe_node["kp"].as<double>();
        }
        if (keyframe_node["kd"]) {
            keyframe.kd = keyframe_node["kd"].as<double>();
        }
        if (keyframe_node["motor_positions"]) {
            for (const auto& item : keyframe_node["motor_positions"]) {
                keyframe.motor_positions[item.first.as<std::string>()] = item.second.as<double>();
            }
        }
        if (keyframe_node["p_gains"]) {
            for (const auto& item : keyframe_node["p_gains"]) {
                keyframe.p_gains[item.first.as<std::string>()] = item.second.as<double>();
            }
        }
        if (keyframe_node["d_gains"]) {
            for (const auto& item : keyframe_node["d_gains"]) {
                keyframe.d_gains[item.first.as<std::string>()] = item.second.as<double>();
            }
        }
        keyframes_.push_back(std::move(keyframe));
    }
}

KeyframeStepResult KeyframeRunner::step(bool reset, double server_time) {
    if (keyframes_.empty()) {
        throw std::runtime_error("KeyframeRunner has no keyframes");
    }
    if (reset) {
        keyframe_step_ = 0;
        keyframe_start_time_ = server_time;
    }

    const Keyframe& keyframe = keyframes_.at(keyframe_step_);
    const double frame_kp = keyframe.kp.value_or(default_kp_);
    const double frame_kd = keyframe.kd.value_or(default_kd_);

    robot::JointTargets joint_targets;
    for (const auto& [logical_name, position] : keyframe.motor_positions) {
        const double kp = keyframe.p_gains.count(logical_name) ? keyframe.p_gains.at(logical_name) : frame_kp;
        const double kd = keyframe.d_gains.count(logical_name) ? keyframe.d_gains.at(logical_name) : frame_kd;

        if (has_symmetry_) {
            const auto& group = robot_model_.symmetry_group(logical_name);
            for (std::size_t i = 0; i < group.motor_names.size(); ++i) {
                const bool flip = group.invert_direction && i == 0;
                joint_targets.push_back({
                    group.motor_names[i],
                    flip ? position : -position,
                    0.0,
                    kp,
                    kd,
                    0.0,
                });
            }
        } else {
            joint_targets.push_back({
                logical_name,
                position,
                0.0,
                kp,
                kd,
                0.0,
            });
        }
    }

    bool finished = false;
    if (server_time - keyframe_start_time_ >= keyframe.delta) {
        keyframe_start_time_ = server_time;
        ++keyframe_step_;
        if (keyframe_step_ >= keyframes_.size()) {
            keyframe_step_ = keyframes_.size() - 1;
            finished = true;
        }
    }

    return {joint_targets, finished};
}

}  // namespace behavior
