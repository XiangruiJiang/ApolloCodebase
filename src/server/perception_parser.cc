// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 The Hong Kong University of Science and Technology (Guangzhou), Humanoid Computing & Learning Lab.

#include "src/server/perception_parser.h"

#include <charconv>
#include <cmath>
#include <cctype>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace server {

namespace {

constexpr std::string_view kWhitespace = " \t\n\r";

std::vector<std::string_view> split_top_level_groups(std::string_view input) {
    std::vector<std::string_view> groups;
    int depth = 0;
    std::size_t start = std::string_view::npos;

    for (std::size_t i = 0; i < input.size(); ++i) {
        if (input[i] == '(') {
            if (depth == 0) {
                start = i;
            }
            ++depth;
        } else if (input[i] == ')') {
            --depth;
            if (depth == 0 && start != std::string_view::npos) {
                groups.push_back(input.substr(start, i - start + 1));
                start = std::string_view::npos;
            }
        }
    }
    return groups;
}

std::string_view trim(std::string_view value) {
    const std::size_t begin = value.find_first_not_of(kWhitespace);
    if (begin == std::string_view::npos) {
        return {};
    }
    const std::size_t end = value.find_last_not_of(kWhitespace);
    return value.substr(begin, end - begin + 1);
}

std::vector<std::string_view> split_ws(std::string_view value) {
    std::vector<std::string_view> tokens;
    std::size_t i = 0;
    while (i < value.size()) {
        while (i < value.size() && std::isspace(static_cast<unsigned char>(value[i]))) {
            ++i;
        }
        const std::size_t start = i;
        while (i < value.size() && !std::isspace(static_cast<unsigned char>(value[i]))) {
            ++i;
        }
        if (start < i) {
            tokens.push_back(value.substr(start, i - start));
        }
    }
    return tokens;
}

double to_double(std::string_view value) {
    double result = 0.0;
    auto [ptr, ec] = std::from_chars(value.data(), value.data() + value.size(), result);
    if (ec == std::errc{} && ptr == value.data() + value.size()) {
        return result;
    }
    return std::stod(std::string(value));
}

int to_int(std::string_view value) {
    int result = 0;
    auto [ptr, ec] = std::from_chars(value.data(), value.data() + value.size(), result);
    if (ec == std::errc{} && ptr == value.data() + value.size()) {
        return result;
    }
    return std::stoi(std::string(value));
}

std::array<double, 3> parse_three_doubles(std::string_view values, const char* context) {
    const auto tokens = split_ws(values);
    if (tokens.size() != 3U) {
        throw std::invalid_argument(std::string("Expected 3 values for ") + context);
    }
    return {to_double(tokens[0]), to_double(tokens[1]), to_double(tokens[2])};
}

PolarObservation parse_polar_observation(std::string_view values) {
    const auto parsed = parse_three_doubles(values, "polar observation");
    return {parsed[0], parsed[1], parsed[2]};
}

int server_cycle_from_time(double server_time) {
    return static_cast<int>(std::floor(server_time * 50.0 + 0.1));
}

std::optional<std::string_view> match_single_group_value(std::string_view input, std::string_view key) {
    std::size_t search_from = 0;
    while (search_from + key.size() + 2 < input.size()) {
        const auto pos = input.find('(', search_from);
        if (pos == std::string_view::npos) {
            return std::nullopt;
        }
        if (pos + 1 + key.size() < input.size() &&
            input.compare(pos + 1, key.size(), key) == 0 &&
            input[pos + 1 + key.size()] == ' ') {
            const auto value_start = pos + 2 + key.size();
            const auto value_end = input.find(')', value_start);
            if (value_end == std::string_view::npos) {
                return std::nullopt;
            }
            return trim(input.substr(value_start, value_end - value_start));
        }
        search_from = pos + 1;
    }
    return std::nullopt;
}

std::string_view match_value_or(std::string_view input, std::string_view key, std::string_view fallback) {
    const auto value = match_single_group_value(input, key);
    return value.has_value() ? *value : fallback;
}

std::optional<std::string> match_value_string(std::string_view input, std::string_view key) {
    const auto value = match_single_group_value(input, key);
    if (!value.has_value()) {
        return std::nullopt;
    }
    return std::string(*value);
}

void split_tag_inner(std::string_view content, std::string_view& tag, std::string_view& inner) {
    const auto space = content.find_first_of(kWhitespace);
    if (space == std::string_view::npos) {
        tag = content;
        inner = {};
    } else {
        tag = content.substr(0, space);
        inner = trim(content.substr(space + 1));
    }
}

std::string_view group_content(std::string_view group) {
    return group.substr(1, group.size() - 2);
}

bool is_well_formed_group(std::string_view group) {
    return group.size() >= 3 && group.front() == '(' && group.back() == ')';
}

std::vector<std::pair<int, std::string>> parse_microphone_messages(std::string_view inner) {
    std::vector<std::pair<int, std::string>> messages;
    for (std::string_view group : split_top_level_groups(inner)) {
        if (!is_well_formed_group(group)) {
            continue;
        }
        const auto tokens = split_ws(group_content(group));
        if (tokens.size() == 2) {
            messages.emplace_back(to_int(tokens[0]), std::string(tokens[1]));
        }
    }
    return messages;
}

}  // namespace

PerceptionFrame PerceptionParser::parse(const std::string& message) const {
    PerceptionFrame frame;

    for (std::string_view group : split_top_level_groups(message)) {
        if (!is_well_formed_group(group)) {
            throw std::invalid_argument("Malformed top-level perception group: " + std::string(group));
        }
        std::string_view tag;
        std::string_view inner;
        split_tag_inner(group_content(group), tag, inner);

        if (tag == "time") {
            const auto now = match_single_group_value(inner, "now");
            if (!now.has_value()) {
                throw std::invalid_argument("time group missing now value");
            }
            frame.server_time = to_double(*now);
            frame.server_cycle = server_cycle_from_time(frame.server_time);
            continue;
        }

        if (tag == "GS") {
            GameStatePerception gs;
            gs.play_time = to_double(match_value_or(inner, "t", "0"));
            gs.play_mode = std::string(match_value_or(inner, "pm", ""));
            gs.team_left = match_value_string(inner, "tl");
            gs.team_right = match_value_string(inner, "tr");
            gs.score_left = to_int(match_value_or(inner, "sl", "0"));
            gs.score_right = to_int(match_value_or(inner, "sr", "0"));
            frame.game_state = gs;
            continue;
        }

        if (tag == "HJ") {
            JointStatePerception joint;
            joint.name = std::string(match_value_or(inner, "n", ""));
            joint.ax_deg = to_double(match_value_or(inner, "ax", "0"));
            joint.vx_deg_s = to_double(match_value_or(inner, "vx", "0"));
            frame.joints.push_back(joint);
            continue;
        }

        if (tag == "GYR") {
            GyroPerception gyro;
            gyro.name = std::string(match_value_or(inner, "n", ""));
            gyro.rot_deg_s = parse_three_doubles(match_value_or(inner, "rt", "0 0 0"), "GYR/rt");
            frame.gyro = gyro;
            continue;
        }

        if (tag == "ACC") {
            AccelerometerPerception accel;
            accel.name = std::string(match_value_or(inner, "n", ""));
            accel.accel_mps2 = parse_three_doubles(match_value_or(inner, "a", "0 0 0"), "ACC/a");
            frame.accel = accel;
            continue;
        }

        if (tag == "pos") {
            PositionPerception position;
            position.name = std::string(match_value_or(inner, "n", ""));
            position.xyz_m = parse_three_doubles(match_value_or(inner, "p", "0 0 0"), "pos/p");
            frame.position = position;
            continue;
        }

        if (tag == "quat") {
            QuaternionPerception quat;
            quat.name = std::string(match_value_or(inner, "n", ""));
            const auto tokens = split_ws(match_value_or(inner, "q", "1 0 0 0"));
            if (tokens.size() != 4U) {
                throw std::invalid_argument("quat/q must contain four values");
            }
            quat.wxyz = {to_double(tokens[0]), to_double(tokens[1]), to_double(tokens[2]), to_double(tokens[3])};
            frame.orientation = quat;
            continue;
        }

        if (tag == "MIC") {
            const auto first_space = inner.find_first_of(" \t");
            const std::string_view payload =
                first_space == std::string_view::npos ? std::string_view{} : inner.substr(first_space + 1);
            for (auto& message_pair : parse_microphone_messages(payload)) {
                frame.microphone_messages.push_back({message_pair.first, std::move(message_pair.second)});
            }
            continue;
        }

        if (tag == "See") {
            for (std::string_view object_group : split_top_level_groups(inner)) {
                if (!is_well_formed_group(object_group)) {
                    throw std::invalid_argument("Malformed See object: " + std::string(object_group));
                }
                std::string_view object_type;
                std::string_view object_inner;
                split_tag_inner(group_content(object_group), object_type, object_inner);

                if (object_type == "P") {
                    PlayerVisionDetection player;
                    player.team_name = std::string(match_value_or(object_inner, "team", ""));
                    player.player_number = to_int(match_value_or(object_inner, "id", "0"));
                    for (std::string_view child : split_top_level_groups(object_inner)) {
                        if (!is_well_formed_group(child)) {
                            continue;
                        }
                        const auto child_content = group_content(child);
                        const auto child_space = child_content.find_first_of(kWhitespace);
                        if (child_space == std::string_view::npos) {
                            continue;
                        }
                        const std::string_view marker_name = child_content.substr(0, child_space);
                        if (marker_name == "team" || marker_name == "id") {
                            continue;
                        }
                        const auto pol_value = match_single_group_value(child_content, "pol");
                        if (!pol_value.has_value()) {
                            continue;
                        }
                        player.markers.push_back({std::string(marker_name), parse_polar_observation(*pol_value)});
                    }
                    frame.vision.players.push_back(player);
                    continue;
                }

                const auto pol = match_single_group_value(object_inner, "pol");
                if (!pol.has_value()) {
                    throw std::invalid_argument("See object missing pol: " + std::string(object_group));
                }
                const PolarObservation polar = parse_polar_observation(*pol);
                if (object_type == "B") {
                    frame.vision.ball = BallVisionDetection{polar};
                } else {
                    frame.vision.landmarks.push_back({std::string(object_type), polar});
                }
            }
            continue;
        }
    }

    return frame;
}

}  // namespace server
