// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 The Hong Kong University of Science and Technology (Guangzhou), Humanoid Computing & Learning Lab.

#include "src/robot/t1_robot_model.h"

#include "src/math/math_utils.h"

#include <algorithm>
#include <stdexcept>

namespace robot {

namespace {

constexpr double kDefaultKp = 10.0;
constexpr double kDefaultKd = 0.1;

}  // namespace

T1RobotModel::T1RobotModel()
    : readable_joint_names_{
          "Head_yaw",
          "Head_pitch",
          "Left_Shoulder_Pitch",
          "Left_Shoulder_Roll",
          "Left_Elbow_Pitch",
          "Left_Elbow_Yaw",
          "Right_Shoulder_Pitch",
          "Right_Shoulder_Roll",
          "Right_Elbow_Pitch",
          "Right_Elbow_Yaw",
          "Waist",
          "Left_Hip_Pitch",
          "Left_Hip_Roll",
          "Left_Hip_Yaw",
          "Left_Knee_Pitch",
          "Left_Ankle_Pitch",
          "Left_Ankle_Roll",
          "Right_Hip_Pitch",
          "Right_Hip_Roll",
          "Right_Hip_Yaw",
          "Right_Knee_Pitch",
          "Right_Ankle_Pitch",
          "Right_Ankle_Roll",
      },
      actuator_names_{
          "he1",  "he2",  "lae1", "lae2", "lae3", "lae4", "rae1", "rae2",
          "rae3", "rae4", "te1",  "lle1", "lle2", "lle3", "lle4", "lle5",
          "lle6", "rle1", "rle2", "rle3", "rle4", "rle5", "rle6",
      } {
    const auto compute_gains = [](const std::string& name) -> GainPair {
        if (name == "Head_yaw")  return {10.0, 1.0};
        if (name == "Head_pitch") return {20.0, 1.0};
        if (name == "Waist")     return {85.0, 5.0};
        if (name.find("Shoulder") != std::string::npos)    return {45.0, 2.5};
        if (name.find("Elbow") != std::string::npos)       return {30.0, 1.2};
        if (name.find("Hip_Pitch") != std::string::npos)   return {130.0, 10.0};
        if (name.find("Hip_Roll") != std::string::npos)    return {90.0, 8.0};
        if (name.find("Hip_Yaw") != std::string::npos)     return {70.0, 3.0};
        if (name.find("Knee") != std::string::npos)        return {140.0, 6.0};
        if (name.find("Ankle_Pitch") != std::string::npos) return {45.0, 2.0};
        if (name.find("Ankle_Roll") != std::string::npos)  return {40.0, 1.8};
        return {kDefaultKp, kDefaultKd};
    };

    for (std::size_t i = 0; i < readable_joint_names_.size(); ++i) {
        const auto& name = readable_joint_names_[i];
        readable_to_actuator_.emplace(name, actuator_names_[i]);
        joint_gains_.emplace(name, compute_gains(name));
        joint_order_.emplace(name, i);
    }

    symmetry_groups_ = {
        {"Head_yaw",       {{"Head_yaw"},                                       false}},
        {"Head_pitch",     {{"Head_pitch"},                                     false}},
        {"Shoulder_Pitch", {{"Left_Shoulder_Pitch", "Right_Shoulder_Pitch"},    false}},
        {"Shoulder_Roll",  {{"Left_Shoulder_Roll",  "Right_Shoulder_Roll"},     true}},
        {"Elbow_Pitch",    {{"Left_Elbow_Pitch",    "Right_Elbow_Pitch"},       false}},
        {"Elbow_Yaw",      {{"Left_Elbow_Yaw",      "Right_Elbow_Yaw"},         true}},
        {"Waist",          {{"Waist"},                                          false}},
        {"Hip_Pitch",      {{"Left_Hip_Pitch",      "Right_Hip_Pitch"},         false}},
        {"Hip_Roll",       {{"Left_Hip_Roll",       "Right_Hip_Roll"},          true}},
        {"Hip_Yaw",        {{"Left_Hip_Yaw",        "Right_Hip_Yaw"},           true}},
        {"Knee_Pitch",     {{"Left_Knee_Pitch",     "Right_Knee_Pitch"},        false}},
        {"Ankle_Pitch",    {{"Left_Ankle_Pitch",    "Right_Ankle_Pitch"},       false}},
        {"Ankle_Roll",     {{"Left_Ankle_Roll",     "Right_Ankle_Roll"},        true}},
    };
}

const std::vector<std::string>& T1RobotModel::readable_joint_names() const {
    return readable_joint_names_;
}

const std::vector<std::string>& T1RobotModel::actuator_names() const {
    return actuator_names_;
}

const std::string& T1RobotModel::actuator_name_for_joint(const std::string& readable_joint_name) const {
    const auto it = readable_to_actuator_.find(readable_joint_name);
    if (it == readable_to_actuator_.end()) {
        throw std::invalid_argument("Unknown T1 readable joint name: " + readable_joint_name);
    }
    return it->second;
}

double T1RobotModel::joint_kp(const std::string& readable_joint_name) const {
    const auto it = joint_gains_.find(readable_joint_name);
    return it != joint_gains_.end() ? it->second.kp : kDefaultKp;
}

double T1RobotModel::joint_kd(const std::string& readable_joint_name) const {
    const auto it = joint_gains_.find(readable_joint_name);
    return it != joint_gains_.end() ? it->second.kd : kDefaultKd;
}

const T1RobotModel::SymmetryGroup& T1RobotModel::symmetry_group(const std::string& logical_joint_name) const {
    const auto it = symmetry_groups_.find(logical_joint_name);
    if (it == symmetry_groups_.end()) {
        throw std::invalid_argument("Unknown T1 symmetry group: " + logical_joint_name);
    }
    return it->second;
}

T1RobotModel::HeadTargets T1RobotModel::clamp_head_targets(double yaw_deg, double pitch_deg) const {
    return {std::clamp(yaw_deg, -90.0, 90.0), std::clamp(pitch_deg, -20.0, 70.0)};
}

world::Vec3 T1RobotModel::camera_position_torso(double head_yaw_deg, double head_pitch_deg) const {
    const world::Vec3 head_base_offset_torso{0.0625, 0.0, 0.243};
    const world::Vec3 head_pitch_offset{0.0, 0.0, 0.06185};
    const world::Vec3 camera_offset{0.05, 0.0, 0.12};

    const world::Vec3 yawed_pitch_offset = math::rotate_z(head_pitch_offset, head_yaw_deg);
    const world::Vec3 pitched_camera_offset = math::rotate_y(camera_offset, head_pitch_deg);
    const world::Vec3 yawed_camera_offset = math::rotate_z(pitched_camera_offset, head_yaw_deg);
    return math::vec3_add(math::vec3_add(head_base_offset_torso, yawed_pitch_offset), yawed_camera_offset);
}

std::optional<std::size_t> T1RobotModel::joint_order_index(const std::string& readable_joint_name) const {
    const auto it = joint_order_.find(readable_joint_name);
    if (it == joint_order_.end()) {
        return std::nullopt;
    }
    return it->second;
}

}  // namespace robot
