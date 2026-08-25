// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 The Hong Kong University of Science and Technology (Guangzhou), Humanoid Computing & Learning Lab.

#include "src/server/action_encoder.h"

#include "src/robot/joint_targets.h"
#include "src/robot/t1_robot_model.h"

#include <algorithm>
#include <array>
#include <charconv>
#include <optional>
#include <stdexcept>
#include <string_view>

namespace server {

namespace {

constexpr char kBase64Alphabet[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "abcdefghijklmnopqrstuvwxyz"
    "0123456789+/";

// std::to_chars in shortest-roundtrip mode emits the canonical short form
// (e.g. "1" for 1.0, "0.1" exact). No locale, no allocation.
std::string format_number(double value) {
    std::array<char, 32> buffer;
    const auto result = std::to_chars(buffer.data(), buffer.data() + buffer.size(), value);
    if (result.ec != std::errc{}) {
        throw std::runtime_error("format_number: to_chars failed");
    }
    std::string_view view(buffer.data(), result.ptr - buffer.data());
    if (view == "-0") {
        return "0";
    }
    return std::string(view);
}

int decode_base64_char(char ch) {
    if (ch >= 'A' && ch <= 'Z') return ch - 'A';
    if (ch >= 'a' && ch <= 'z') return ch - 'a' + 26;
    if (ch >= '0' && ch <= '9') return ch - '0' + 52;
    if (ch == '+') return 62;
    if (ch == '/') return 63;
    if (ch == '=') return -1;
    throw std::invalid_argument("Invalid base64 character");
}

}  // namespace

std::string encode_base64(const std::vector<std::uint8_t>& bytes) {
    std::string output;
    output.reserve(((bytes.size() + 2U) / 3U) * 4U);

    std::size_t index = 0;
    while (index + 3U <= bytes.size()) {
        const std::uint32_t chunk = (static_cast<std::uint32_t>(bytes[index]) << 16U) |
                                    (static_cast<std::uint32_t>(bytes[index + 1U]) << 8U) |
                                    static_cast<std::uint32_t>(bytes[index + 2U]);
        output.push_back(kBase64Alphabet[(chunk >> 18U) & 0x3FU]);
        output.push_back(kBase64Alphabet[(chunk >> 12U) & 0x3FU]);
        output.push_back(kBase64Alphabet[(chunk >> 6U) & 0x3FU]);
        output.push_back(kBase64Alphabet[chunk & 0x3FU]);
        index += 3U;
    }

    const std::size_t remaining = bytes.size() - index;
    if (remaining == 1U) {
        const std::uint32_t chunk = static_cast<std::uint32_t>(bytes[index]) << 16U;
        output.push_back(kBase64Alphabet[(chunk >> 18U) & 0x3FU]);
        output.push_back(kBase64Alphabet[(chunk >> 12U) & 0x3FU]);
        output.push_back('=');
        output.push_back('=');
    } else if (remaining == 2U) {
        const std::uint32_t chunk = (static_cast<std::uint32_t>(bytes[index]) << 16U) |
                                    (static_cast<std::uint32_t>(bytes[index + 1U]) << 8U);
        output.push_back(kBase64Alphabet[(chunk >> 18U) & 0x3FU]);
        output.push_back(kBase64Alphabet[(chunk >> 12U) & 0x3FU]);
        output.push_back(kBase64Alphabet[(chunk >> 6U) & 0x3FU]);
        output.push_back('=');
    }

    return output;
}

std::vector<std::uint8_t> decode_base64(const std::string& input) {
    if (input.size() % 4U != 0U) {
        throw std::invalid_argument("base64 input length must be a multiple of 4");
    }
    std::vector<std::uint8_t> output;
    output.reserve(input.size() / 4U * 3U);
    for (std::size_t i = 0; i < input.size(); i += 4U) {
        const int a = decode_base64_char(input.at(i));
        const int b = decode_base64_char(input.at(i + 1U));
        const int c = decode_base64_char(input.at(i + 2U));
        const int d = decode_base64_char(input.at(i + 3U));

        output.push_back(static_cast<std::uint8_t>((a << 2) | (b >> 4)));
        if (c >= 0) {
            output.push_back(static_cast<std::uint8_t>(((b & 0x0F) << 4) | (c >> 2)));
        }
        if (d >= 0 && c >= 0) {
            output.push_back(static_cast<std::uint8_t>(((c & 0x03) << 6) | d));
        }
    }
    return output;
}

std::string ActionEncoder::make_init(const std::string& team_name, int player_number) {
    if (team_name.empty()) {
        throw std::invalid_argument("team_name must not be empty");
    }
    if (player_number <= 0) {
        throw std::invalid_argument("player_number must be positive");
    }

    return "(init T1 " + team_name + " " + std::to_string(player_number) + ")";
}

std::string ActionEncoder::encode_beam(double x_m, double y_m, double yaw_deg) {
    return "(beam " + format_number(x_m) + " " + format_number(y_m) + " " +
           format_number(yaw_deg) + ")";
}

std::string ActionEncoder::encode_motor_node(
    const std::string& name,
    double q_deg,
    double dq_deg,
    double kp,
    double kd,
    double tau) {
    if (name.empty()) {
        throw std::invalid_argument("motor name must not be empty");
    }

    return "(" + name + " " + format_number(q_deg) + " " + format_number(dq_deg) + " " +
           format_number(kp) + " " + format_number(kd) + " " + format_number(tau) + ")";
}

std::vector<std::string> ActionEncoder::encode_motor_actions(
    const robot::JointTargets& joint_targets,
    const robot::T1RobotModel& robot_model) {
    struct OrderedTarget {
        std::size_t order_index;
        const robot::JointTarget* target;
    };

    std::vector<OrderedTarget> ordered_targets;
    ordered_targets.reserve(joint_targets.size());

    for (const robot::JointTarget& target : joint_targets) {
        const std::optional<std::size_t> order_index = robot_model.joint_order_index(target.joint_name);
        if (!order_index.has_value()) {
            throw std::invalid_argument("Unknown T1 joint in JointTargets: " + target.joint_name);
        }
        ordered_targets.push_back({*order_index, &target});
    }

    std::sort(
        ordered_targets.begin(),
        ordered_targets.end(),
        [](const OrderedTarget& lhs, const OrderedTarget& rhs) {
            return lhs.order_index < rhs.order_index;
        });

    std::vector<std::string> encoded;
    encoded.reserve(ordered_targets.size());
    for (const OrderedTarget& ordered : ordered_targets) {
        const robot::JointTarget& target = *ordered.target;
        encoded.push_back(encode_motor_node(
            robot_model.actuator_name_for_joint(target.joint_name),
            target.q_deg,
            target.dq_deg,
            target.kp,
            target.kd,
            target.tau));
    }

    return encoded;
}

std::string ActionEncoder::encode_spk(const std::vector<std::uint8_t>& payload_bytes) {
    return "(SPK say 100 " + encode_base64(payload_bytes) + ")";
}

std::string ActionEncoder::finish_frame(const std::vector<std::string>& nodes) {
    std::string frame;
    for (const std::string& node : nodes) {
        frame += node;
    }
    frame += "(syn)";
    return frame;
}

}  // namespace server
