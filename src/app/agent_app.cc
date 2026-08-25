// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 The Hong Kong University of Science and Technology (Guangzhou), Humanoid Computing & Learning Lab.

#include "src/app/agent_app.h"

#include "src/comm/team_comm_codec.h"
#include "src/decision/role_behaviors.h"
#include "src/math/math_utils.h"
#include "src/server/action_encoder.h"
#include "src/world/frame_normalizer.h"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

namespace app {

namespace {

bool infer_is_left_team(const server::PerceptionFrame& frame, const std::string& team_name, const world::WorldState& world_state) {
    if (frame.game_state.has_value()) {
        const auto& gs = *frame.game_state;
        if (gs.team_left.has_value() && gs.team_left.value() == team_name) {
            return true;
        }
        if (gs.team_right.has_value() && gs.team_right.value() == team_name) {
            return false;
        }
    }
    return world_state.snapshot().is_left_team.value_or(true);
}

void normalize_frame_for_team(server::PerceptionFrame& frame, bool is_left_team) {
    if (is_left_team) {
        return;
    }
    if (frame.position.has_value()) {
        frame.position->xyz_m = world::FrameNormalizer::normalize_position(frame.position->xyz_m, false);
    }
    if (frame.orientation.has_value()) {
        frame.orientation->wxyz = world::FrameNormalizer::normalize_quaternion_wxyz(frame.orientation->wxyz, false);
    }
}

}  // namespace

AgentApp::AgentApp(RuntimeConfig config)
    : config_(std::move(config)),
      world_state_(config_.team_name, config_.player_number, 7),
      motion_manager_(config_),
      team_comm_manager_(config_.team_name) {}

AgentApp::AgentApp(RuntimeConfig config, std::unique_ptr<server::TcpLpmClient> client)
    : config_(std::move(config)),
      world_state_(config_.team_name, config_.player_number, 7),
      motion_manager_(config_),
      team_comm_manager_(config_.team_name),
      client_(std::move(client)) {}

int AgentApp::run() {
    return run_for_cycles(std::numeric_limits<std::size_t>::max());
}

int AgentApp::run_for_cycles(std::size_t cycles) {
    if (cycles == 0U) {
        return 0;
    }
    if (!client_) {
        client_ = std::make_unique<server::TcpLpmClient>(config_.host, config_.port);
    }
    client_->connect();
    if (!init_sent_) {
        client_->send_message(server::ActionEncoder::make_init(config_.team_name, config_.player_number));
        init_sent_ = true;
    }

    std::size_t processed = 0;
    while (!shutdown_requested_ && processed < cycles) {
        const std::string message = client_->receive_message();
        const std::string response = process_perception_message(message);
        client_->send_message(response);
        ++processed;
    }
    return 0;
}

std::string AgentApp::process_perception_message(const std::string& message) {
    server::PerceptionFrame frame = perception_parser_.parse(message);
    const bool is_left_team = infer_is_left_team(frame, config_.team_name, world_state_);
    normalize_frame_for_team(frame, is_left_team);
    world_state_.update_from_perception(frame, robot_model_, is_left_team);

    for (const auto& mic : frame.microphone_messages) {
        try {
            const auto payload = server::decode_base64(mic.message_b64);
            if (payload.size() == comm::TeamCommCodec::kPacketSizeBytes) {
                team_comm_manager_.ingest(comm::TeamCommCodec::decode(payload), frame.server_cycle);
            }
        } catch (const std::exception&) {
        }
    }
    world_state_.set_team_comm_snapshot(team_comm_manager_.make_snapshot(frame.server_cycle));

    const world::WorldSnapshot& snapshot = world_state_.snapshot();
    const auto command = decision_manager_.decide(snapshot);
    const bool reset = last_command_variant_index_ != command.index();
    last_command_variant_index_ = command.index();

    std::vector<std::string> nodes;
    if (const auto* beam = std::get_if<decision::BeamCommand>(&command)) {
        // Beam is the only absolute-coordinate output. The agent works in a
        // canonical frame (own goal at -x) obtained by a 180-degree rotation of
        // the global frame for the right team, but the server's beam contract
        // maps the right team by REFLECTION (x = abs(x) * side, y unchanged,
        // theta + pi). Those agree on x and yaw but disagree on y, so the right
        // team's canonical beam y must be negated to land where intended.
        const double beam_y = is_left_team ? beam->y_m : -beam->y_m;
        nodes.push_back(server::ActionEncoder::encode_beam(beam->x_m, beam_y, beam->yaw_deg));
        world_state_.set_has_beamed(true);
        last_active_motion_ = "Beam";
    } else {
        const auto motion_result = motion_manager_.step(snapshot, command, reset);
        last_active_motion_ = motion_result.active_motion;
        if (motion_result.handled) {
            const auto motor_nodes = server::ActionEncoder::encode_motor_actions(motion_result.joint_targets, robot_model_);
            nodes.insert(nodes.end(), motor_nodes.begin(), motor_nodes.end());
        }
    }

    if (team_comm_manager_.is_send_slot(config_.player_number, frame.server_cycle)) {
        const auto packet = team_comm_manager_.make_packet(
            snapshot,
            decision::current_role_from_blackboard(decision_manager_.blackboard()));
        nodes.push_back(server::ActionEncoder::encode_spk(comm::TeamCommCodec::encode(packet)));
    }

    return server::ActionEncoder::finish_frame(nodes);
}

void AgentApp::shutdown() {
    shutdown_requested_ = true;
}

const RuntimeConfig& AgentApp::config() const {
    return config_;
}

bool AgentApp::shutdown_requested() const {
    return shutdown_requested_;
}

const std::string& AgentApp::last_active_motion() const {
    return last_active_motion_;
}

}  // namespace app
