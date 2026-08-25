// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 The Hong Kong University of Science and Technology (Guangzhou), Humanoid Computing & Learning Lab.

#include "src/behavior/walk_runner.h"

#include "src/decision/role_manager.h"
#include "src/math/math_utils.h"
#include "src/behavior/policy_common.h"
#include "src/world/frame_normalizer.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace behavior {

namespace {

using math::deg_to_rad;
using math::kPi;
using math::norm2;
using math::quaternion_conjugate;
using math::rotate_2d;
using math::rotate_vec_by_quaternion;
using math::vector_angle_deg;

std::optional<robot::T1RobotModel::HeadTargets> update_head_tracker(
    behavior::WalkRunner::HeadTrackerState& state,
    const world::WorldSnapshot& snapshot,
    const robot::T1RobotModel& robot_model,
    std::optional<int> role_id) {
    constexpr double lock_distance = 1.0;
    constexpr double lock_hold_time = 0.6;
    constexpr double search_sweep_deg = 70.0;
    constexpr double search_period = 3.0;
    constexpr double sweep_min_deg = -90.0;
    constexpr double sweep_max_deg = 90.0;
    constexpr double sweep_speed_deg_per_sec = 60.0;
    constexpr double sweep_pitch_deg = 0.0;

    // Only the GK (ROLE_GK = 0) tracks the ball with its head; every other
    // role sweeps the field. Roles are 0..6 in this codebase (see
    // decision::RoleManager), so there is no special "role 10" anymore.
    if (role_id.has_value() && *role_id != decision::RoleManager::ROLE_GK) {
        const double sweep_range = sweep_max_deg - sweep_min_deg;
        const double period = sweep_range / sweep_speed_deg_per_sec * 2.0;
        const double t = std::fmod(snapshot.server_time, period);
        double yaw = 0.0;
        if (t < period / 2.0) {
            const double progress = t / (period / 2.0);
            yaw = sweep_min_deg + progress * sweep_range;
        } else {
            const double progress = (t - period / 2.0) / (period / 2.0);
            yaw = sweep_max_deg - progress * sweep_range;
        }
        return robot_model.clamp_head_targets(yaw, sweep_pitch_deg);
    }

    if (snapshot.ball.visible) {
        const std::array<double, 3> ball_vec_world{
            snapshot.ball.position_m[0] - snapshot.self.position_m[0],
            snapshot.ball.position_m[1] - snapshot.self.position_m[1],
            snapshot.ball.position_m[2] - snapshot.self.position_m[2],
        };
        if (norm2({ball_vec_world[0], ball_vec_world[1]}) <= lock_distance) {
            if (!state.last_head_target_deg.has_value()) {
                return std::nullopt;
            }
            const auto target = state.last_head_target_deg.value();
            return robot_model.clamp_head_targets(target[0], target[1]);
        }

        const auto body_vec = rotate_vec_by_quaternion(ball_vec_world, quaternion_conjugate(snapshot.self.orientation_wxyz));
        const double horiz = std::sqrt(body_vec[0] * body_vec[0] + body_vec[1] * body_vec[1]);
        if (horiz < 1e-6) {
            return std::nullopt;
        }
        const double yaw = std::atan2(body_vec[1], body_vec[0]) * 180.0 / kPi;
        const double pitch = -std::atan2(body_vec[2], horiz) * 180.0 / kPi;
        state.last_ball_seen_time = snapshot.server_time;
        state.last_head_target_deg = std::array<double, 2>{yaw, pitch};
        return robot_model.clamp_head_targets(yaw, pitch);
    }

    if (!state.last_head_target_deg.has_value() || !state.last_ball_seen_time.has_value()) {
        return std::nullopt;
    }

    const double time_since_seen = snapshot.server_time - state.last_ball_seen_time.value();
    if (time_since_seen <= lock_hold_time) {
        const auto target = state.last_head_target_deg.value();
        return robot_model.clamp_head_targets(target[0], target[1]);
    }

    const double phase =
        2.0 * kPi * std::fmod(snapshot.server_time, search_period) / search_period;
    const double yaw = search_sweep_deg * std::sin(phase);
    return robot_model.clamp_head_targets(yaw, sweep_pitch_deg);
}

}  // namespace

WalkRunner::WalkRunner(const std::filesystem::path& model_path)
    : session_(model_path, OnnxModelContract{{1, 78}, {1, 23}}) {
    step_obs_dim_ = 9 + 3 * static_cast<int>(robot_model_.readable_joint_names().size());
    const auto& input_shape = session_.info().input_shape;
    history_length_ = static_cast<int>(input_shape.at(1) / step_obs_dim_);
    if (history_length_ < 1 || history_length_ * step_obs_dim_ != input_shape.at(1)) {
        throw std::runtime_error("WalkRunner input dimension is incompatible with step observation layout");
    }
    previous_action_.assign(robot_model_.readable_joint_names().size(), 0.0F);
    observation_.assign(static_cast<std::size_t>(history_length_ * step_obs_dim_), 0.0F);
    step_obs_buffer_.assign(static_cast<std::size_t>(step_obs_dim_), 0.0F);
}

std::array<float, 3> WalkRunner::compute_velocity_command(
    const world::WorldSnapshot& snapshot,
    const decision::WalkCommand& command) const {
    std::array<float, 3> velocity{0.0F, 0.0F, 0.0F};
    const double self_yaw_deg =
        world::FrameNormalizer::yaw_deg_from_quaternion_wxyz(snapshot.self.orientation_wxyz);
    if (command.target_absolute) {
        const std::array<double, 2> raw_target{
            command.target_2d_m[0] - snapshot.self.position_m[0],
            command.target_2d_m[1] - snapshot.self.position_m[1],
        };
        const auto local_target = rotate_2d(raw_target, -self_yaw_deg);
        velocity[0] = static_cast<float>(local_target[0]);
        velocity[1] = static_cast<float>(local_target[1]);
    } else {
        velocity[0] = static_cast<float>(command.target_2d_m[0]);
        velocity[1] = static_cast<float>(command.target_2d_m[1]);
    }

    double rel_orientation_deg = 0.0;
    if (!command.orientation_deg.has_value()) {
        const std::array<double, 2> planar{velocity[0], velocity[1]};
        if (norm2(planar) > 0.1) {
            rel_orientation_deg = vector_angle_deg(planar);
        }
    } else if (command.orientation_absolute) {
        rel_orientation_deg = math::normalize_deg(command.orientation_deg.value() - self_yaw_deg);
    } else {
        rel_orientation_deg = math::normalize_deg(command.orientation_deg.value());
    }

    velocity[2] = static_cast<float>(kOrientationToAngVelScale * deg_to_rad(rel_orientation_deg));
    velocity[0] = std::clamp(velocity[0], -0.5F, 1.0F);
    velocity[1] = std::clamp(velocity[1], -0.5F, 0.5F);
    velocity[2] = std::clamp(velocity[2], -0.5F, 0.5F);
    return velocity;
}

void WalkRunner::build_observation(
    const world::WorldSnapshot& snapshot,
    const std::array<float, 3>& velocity_command) {
    std::fill(step_obs_buffer_.begin(), step_obs_buffer_.end(), 0.0F);
    const auto imu = build_imu_obs(snapshot);
    step_obs_buffer_[0] = imu[0];
    step_obs_buffer_[1] = imu[1];
    step_obs_buffer_[2] = imu[2];
    step_obs_buffer_[3] = imu[3];
    step_obs_buffer_[4] = imu[4];
    step_obs_buffer_[5] = imu[5];
    step_obs_buffer_[6] = velocity_command[0];
    step_obs_buffer_[7] = velocity_command[1];
    step_obs_buffer_[8] = velocity_command[2];

    const auto& joint_names = robot_model_.readable_joint_names();
    const std::size_t offset = 9U;
    auto* pos_out = step_obs_buffer_.data() + offset;
    auto* vel_out = pos_out + joint_names.size();
    auto* prev_out = vel_out + joint_names.size();
    fill_joint_obs(pos_out, vel_out, prev_out, snapshot, robot_model_, previous_action_);
    for (std::size_t i = 0; i < 2U; ++i) {
        pos_out[i] = 0.0F;
        vel_out[i] = 0.0F;
        prev_out[i] = 0.0F;
    }
}

robot::JointTargets WalkRunner::decode_action(
    const world::WorldSnapshot& snapshot,
    const std::vector<float>& action,
    std::optional<int> role_id) {
    robot::JointTargets joint_targets = decode_action_base(action, robot_model_, kActionScale);

    const auto head_target = update_head_tracker(head_tracker_state_, snapshot, robot_model_, role_id);
    if (head_target.has_value()) {
        joint_targets[0].q_deg = head_target->yaw_deg;
        joint_targets[1].q_deg = head_target->pitch_deg;
    }

    return joint_targets;
}

WalkStepResult WalkRunner::step(
    const world::WorldSnapshot& snapshot,
    const decision::WalkCommand& command,
    bool reset,
    std::optional<int> role_id) {
    if (reset) {
        std::fill(previous_action_.begin(), previous_action_.end(), 0.0F);
    }

    const auto velocity_command = compute_velocity_command(snapshot, command);
    build_observation(snapshot, velocity_command);
    advance_history(observation_, step_obs_buffer_, step_obs_dim_, history_length_, reset);

    auto action = session_.run(observation_);
    for (float& value : action) {
        value = std::clamp(value, -5.0F, 5.0F);
    }
    previous_action_ = action;

    return {
        observation_,
        action,
        decode_action(snapshot, action, role_id),
    };
}

}  // namespace behavior
