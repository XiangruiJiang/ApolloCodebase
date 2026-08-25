// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 The Hong Kong University of Science and Technology (Guangzhou), Humanoid Computing & Learning Lab.

#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace robot {
class T1RobotModel;
struct JointTarget;
using JointTargets = std::vector<JointTarget>;
}  // namespace robot

namespace server {

/// Encodes high-level actions into RCSSServerMJ protocol nodes.
class ActionEncoder {
public:
    static std::string make_init(const std::string& team_name, int player_number);
    static std::string encode_beam(double x_m, double y_m, double yaw_deg);
    static std::string encode_motor_node(
        const std::string& name,
        double q_deg,
        double dq_deg,
        double kp,
        double kd,
        double tau);
    static std::vector<std::string> encode_motor_actions(
        const robot::JointTargets& joint_targets,
        const robot::T1RobotModel& robot_model);
    static std::string encode_spk(const std::vector<std::uint8_t>& payload_bytes);
    static std::string finish_frame(const std::vector<std::string>& nodes);
};

/// Encodes bytes with RFC 4648 base64.
std::string encode_base64(const std::vector<std::uint8_t>& bytes);
/// Decodes RFC 4648 base64, rejecting malformed input.
std::vector<std::uint8_t> decode_base64(const std::string& input);

}  // namespace server
