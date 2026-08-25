// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 The Hong Kong University of Science and Technology (Guangzhou), Humanoid Computing & Learning Lab.

#include "src/behavior/getup_runner.h"

#include "src/behavior/policy_common.h"

#include <algorithm>
#include <cmath>

namespace behavior {

GetupRunner::GetupRunner(const std::filesystem::path& model_path)
    : session_(model_path, OnnxModelContract{{1, 75}, {1, 23}}) {
    previous_action_.assign(robot_model_.readable_joint_names().size(), 0.0F);
    observation_buffer_.assign(75, 0.0F);
}

void GetupRunner::build_observation(
    const world::WorldSnapshot& snapshot) {
    std::fill(observation_buffer_.begin(), observation_buffer_.end(), 0.0F);

    const auto imu = build_imu_obs(snapshot);
    std::copy(imu.begin(), imu.end(), observation_buffer_.begin());

    const auto& joint_names = robot_model_.readable_joint_names();
    constexpr std::size_t kImuDim = 6;
    fill_joint_obs(
        observation_buffer_.data() + kImuDim,
        observation_buffer_.data() + kImuDim + joint_names.size(),
        observation_buffer_.data() + kImuDim + 2 * joint_names.size(),
        snapshot, robot_model_, previous_action_);
}

robot::JointTargets GetupRunner::decode_action(
    const world::WorldSnapshot& snapshot,
    const std::vector<float>& action) {
    // Relative control: target = current_pos + action * scale
    // (Training uses SettleRelativeJointPositionAction with scale=0.6)
    robot::JointTargets joint_targets;
    const auto& joint_names = robot_model_.readable_joint_names();
    joint_targets.reserve(joint_names.size());

    for (std::size_t i = 0; i < joint_names.size(); ++i) {
        const auto pos_it = snapshot.self.joint_positions_deg.find(joint_names[i]);
        const double current_pos_rad = (pos_it != snapshot.self.joint_positions_deg.end())
            ? math::deg_to_rad(pos_it->second)
            : kDefaultPosRad[i];

        const double target_rad = current_pos_rad
            + static_cast<double>(action[i]) * static_cast<double>(kActionScale);

        joint_targets.push_back({
            joint_names[i],
            math::rad_to_deg(target_rad),
            0.0,
            robot_model_.joint_kp(joint_names[i]),
            robot_model_.joint_kd(joint_names[i]),
            0.0,
        });
    }
    return joint_targets;
}

GetupStepResult GetupRunner::step(
    const world::WorldSnapshot& snapshot,
    bool reset) {
    if (reset) {
        std::fill(previous_action_.begin(), previous_action_.end(), 0.0F);
    }

    build_observation(snapshot);
    auto action = session_.run(observation_buffer_);

    for (float& value : action) {
        value = std::clamp(value, -kActionClip, kActionClip);
    }
    previous_action_ = action;

    // Check upright via projected gravity
    const auto gravity_body = math::rotate_vec_by_quaternion(
        {0.0, 0.0, -1.0},
        math::quaternion_conjugate(snapshot.self.orientation_wxyz));
    const bool upright = gravity_body[2] < kUprightGravityZThreshold;

    return {decode_action(snapshot, action), upright};
}

}  // namespace behavior
