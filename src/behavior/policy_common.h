// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 The Hong Kong University of Science and Technology (Guangzhou), Humanoid Computing & Learning Lab.

#pragma once

#include "src/math/math_utils.h"
#include "src/robot/joint_targets.h"
#include "src/robot/t1_robot_model.h"
#include "src/world/world_snapshot.h"

#include <algorithm>
#include <array>
#include <vector>

namespace behavior {

// Per-joint home pose (rad) for action decoding and observation centering.
// The real-robot deployment (5.5 baseline) uses these values; the mjlab
// HOME_KEYFRAME from t1_constants.py is for the simulator only and does not
// apply here.
inline constexpr std::array<float, 23> kDefaultPosRad{
    0.0F,  0.0F,  0.0F,  -1.4F, 0.0F,  -0.4F, 0.0F, 1.4F,
    0.0F,  0.4F,  0.0F,  -0.2F, 0.0F,   0.0F, 0.4F, -0.2F,
    0.0F, -0.2F,  0.0F,   0.0F, 0.4F,  -0.2F, 0.0F,
};

/// Builds angular-velocity and projected-gravity observations in body frame.
inline std::array<float, 6> build_imu_obs(const world::WorldSnapshot& snapshot) {
    const auto gravity_body =
        math::rotate_vec_by_quaternion({0.0, 0.0, -1.0}, math::quaternion_conjugate(snapshot.self.orientation_wxyz));
    return {
        static_cast<float>(math::deg_to_rad(snapshot.self.gyro_deg_s[0])),
        static_cast<float>(math::deg_to_rad(snapshot.self.gyro_deg_s[1])),
        static_cast<float>(math::deg_to_rad(snapshot.self.gyro_deg_s[2])),
        static_cast<float>(gravity_body[0]),
        static_cast<float>(gravity_body[1]),
        static_cast<float>(gravity_body[2]),
    };
}

/// Writes joint position, velocity, and previous-action policy observations.
inline void fill_joint_obs(
    float* out_pos, float* out_vel, float* out_prev_act,
    const world::WorldSnapshot& snapshot,
    const robot::T1RobotModel& robot_model,
    const std::vector<float>& previous_action) {
    const auto& joint_names = robot_model.readable_joint_names();
    for (std::size_t i = 0; i < joint_names.size(); ++i) {
        const auto pos_it = snapshot.self.joint_positions_deg.find(joint_names[i]);
        const double pos_deg = pos_it == snapshot.self.joint_positions_deg.end() ? 0.0 : pos_it->second;
        out_pos[i] = static_cast<float>(math::deg_to_rad(pos_deg) - kDefaultPosRad[i]);
    }
    for (std::size_t i = 0; i < joint_names.size(); ++i) {
        const auto vel_it = snapshot.self.joint_velocities_deg_s.find(joint_names[i]);
        const double vel_deg = vel_it == snapshot.self.joint_velocities_deg_s.end() ? 0.0 : vel_it->second;
        out_vel[i] = static_cast<float>(math::deg_to_rad(vel_deg));
    }
    for (std::size_t i = 0; i < joint_names.size(); ++i) {
        out_prev_act[i] = previous_action[i];
    }
}

/// Converts normalized policy actions into position-controlled joint targets.
inline robot::JointTargets decode_action_base(
    const std::vector<float>& action,
    const robot::T1RobotModel& robot_model,
    float action_scale = 0.25F) {
    robot::JointTargets joint_targets;
    const auto& joint_names = robot_model.readable_joint_names();
    joint_targets.reserve(joint_names.size());
    for (std::size_t i = 0; i < joint_names.size(); ++i) {
        const double target_rad = static_cast<double>(kDefaultPosRad[i] + action[i] * action_scale);
        joint_targets.push_back({
            joint_names[i],
            math::rad_to_deg(target_rad),
            0.0,
            robot_model.joint_kp(joint_names[i]),
            robot_model.joint_kd(joint_names[i]),
            0.0,
        });
    }
    return joint_targets;
}

/// Appends one observation to a fixed-length, oldest-first history buffer.
inline void advance_history(
    std::vector<float>& history,
    const std::vector<float>& step_obs,
    int step_obs_dim,
    int history_length,
    bool reset) {
    if (reset) {
        for (int h = 0; h < history_length; ++h) {
            std::copy(step_obs.begin(), step_obs.end(), history.begin() + h * step_obs_dim);
        }
    } else {
        if (history_length > 1) {
            std::move(
                history.begin() + step_obs_dim,
                history.end(),
                history.begin());
        }
        std::copy(
            step_obs.begin(),
            step_obs.end(),
            history.end() - step_obs_dim);
    }
}

}  // namespace behavior
