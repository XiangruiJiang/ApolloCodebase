// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 The Hong Kong University of Science and Technology (Guangzhou), Humanoid Computing & Learning Lab.

#include "src/decision/behavior_tree.h"

#include "src/decision/behavior_nodes.h"
#include "src/decision/field_geometry.h"
#include "src/decision/role_behaviors.h"
#include "src/math/math_utils.h"

#include <array>
#include <cmath>
#include <optional>

namespace decision {

namespace {

struct BehaviorContext {
    const world::WorldSnapshot& snapshot;
    Blackboard& blackboard;
    RoleManager& role_manager;
};

using NodeResult = bt::NodeResult<BehaviorContext>;
using NodePtr = bt::NodePtr<BehaviorContext>;

bool is_fallen(const world::WorldSnapshot& snapshot) {
    return snapshot.self.position_m[2] < world::kFallenHeightThresholdM;
}

bool has_teammates(const BehaviorContext& context) {
    return !context.snapshot.teammates.empty();
}

bool needs_beam(const BehaviorContext& context) {
    return (context.snapshot.play_mode_group == world::PlayModeGroup::ActiveBeam ||
            context.snapshot.play_mode_group == world::PlayModeGroup::PassiveBeam) &&
           !context.snapshot.has_beamed;
}

bool is_set_play_mode(const BehaviorContext& context) {
    return context.snapshot.play_mode_group == world::PlayModeGroup::OurKick ||
           context.snapshot.play_mode_group == world::PlayModeGroup::TheirKick;
}

// Humanoid robots walk forward only, so while farther than kFaceTargetRadiusM
// from the destination they must face the direction of travel; once on station
// they face a fixed point (the ball, or the goal to pre-aim a kick). Returns
// std::nullopt when the facing target is degenerate (coincides with the
// reference point), leaving the orientation unset.
constexpr double kFaceTargetRadiusM = 0.5;

std::optional<double> walk_facing_orientation_deg(
    const std::array<double, 2>& self,
    const std::array<double, 2>& destination,
    const std::array<double, 2>& face_when_close) {
    const bool far = math::planar_dist(self, destination) > kFaceTargetRadiusM;
    const std::array<double, 2> from = far ? self : destination;
    const std::array<double, 2> to = far ? destination : face_when_close;
    if (math::sq_dist2(to, from) < 1e-6) {
        return std::nullopt;
    }
    return math::vector_angle_deg(math::vec2_sub(to, from));
}

WalkCommand walk_to_current_role_position(BehaviorContext& context) {
    const int role_id = current_role_from_blackboard(context.blackboard);
    const auto role_pos = context.blackboard.exists(Blackboard::kKeyRolePos)
        ? context.blackboard.get<std::array<double, 2>>(Blackboard::kKeyRolePos)
        : std::array<double, 2>{0.0, 0.0};
    const std::array<double, 2> ball_position{
        context.snapshot.ball.position_m[0],
        context.snapshot.ball.position_m[1],
    };
    const std::array<double, 2> self_position{
        context.snapshot.self.position_m[0],
        context.snapshot.self.position_m[1],
    };
    const auto legal_role_pos =
        field_geometry::legalize_set_play_target(role_pos, ball_position, context.snapshot.play_mode);
    context.blackboard.set(Blackboard::kKeyRolePos, legal_role_pos);

    WalkCommand command;
    command.target_2d_m = legal_role_pos;
    command.target_absolute = true;
    command.role_id = role_id;

    command.orientation_deg =
        walk_facing_orientation_deg(self_position, legal_role_pos, ball_position);
    command.orientation_absolute = true;
    return command;
}

HighLevelCommand make_beam_command(BehaviorContext& context) {
    reset_role_behavior_state();
    // The role manager has not yet run at beam time, so we cannot map this
    // player to a formation slot. Use the player-number pose table, with a
    // deeper front-player pose only when the next kickoff belongs to the other
    // team and the server will enforce center-circle clearance immediately.
    const auto pose =
        context.snapshot.play_mode_group == world::PlayModeGroup::PassiveBeam
            ? field_geometry::player_defensive_kickoff_beam_pose(context.snapshot.player_number)
            : field_geometry::player_beam_pose(context.snapshot.player_number);
    return BeamCommand{pose[0], pose[1], pose[2]};
}

HighLevelCommand make_get_up_command(BehaviorContext&) {
    reset_role_behavior_state();
    return GetUpCommand{};
}

NodeResult compute_formation(BehaviorContext& context) {
    auto [role_id, role_pos] = context.role_manager.get_role(context.snapshot);

    const std::array<double, 2> ball_position{
        context.snapshot.ball.position_m[0],
        context.snapshot.ball.position_m[1],
    };
    const auto legal_role_pos =
        field_geometry::legalize_set_play_target(role_pos, ball_position, context.snapshot.play_mode);
    context.blackboard.set(Blackboard::kKeyCurrentRole, role_id);
    context.blackboard.set(Blackboard::kKeyRolePos, legal_role_pos);
    return NodeResult::success_only();
}

// Deep-pair hold after an opponent kickoff. Players 6 and 7 beam to a back line
// on their kickoff (player_defensive_kickoff_beam_pose) and must keep it for the
// first kKickoffHoldAfterPlayOnS of open play rather than releasing up the pitch
// the instant the ball is in play. The blackboard is cleared every tick, so the
// TheirKickOff -> PlayOn transition is latched in persistent state here (one
// robot per process, mirroring the existing file-scope role state).
constexpr double kKickoffHoldAfterPlayOnS = 3.0;

struct KickoffHoldState {
    world::PlayMode prev_play_mode{world::PlayMode::NotInitialized};
    double hold_until_s{-1.0};
};
KickoffHoldState g_kickoff_hold;

void update_kickoff_hold_state(const world::WorldSnapshot& snapshot) {
    if (g_kickoff_hold.prev_play_mode == world::PlayMode::TheirKickOff &&
        snapshot.play_mode == world::PlayMode::PlayOn) {
        g_kickoff_hold.hold_until_s = snapshot.server_time + kKickoffHoldAfterPlayOnS;
    }
    g_kickoff_hold.prev_play_mode = snapshot.play_mode;
}

bool is_kickoff_hold_active(const BehaviorContext& context) {
    const int player_number = context.snapshot.player_number;
    return (player_number == 6 || player_number == 7) &&
           context.snapshot.play_mode == world::PlayMode::PlayOn &&
           context.snapshot.server_time < g_kickoff_hold.hold_until_s;
}

// During BeforeKickOff the server waits for the kicker to confirm before the
// ball is live. Any teammate who is already inside the center circle (the
// kicker at the center spot, the relay at the perpendicular slot) must hold
// position — otherwise the formation would walk them out before the kickoff
// actually starts.
bool is_before_kickoff_hold(const BehaviorContext& context) {
    if (context.snapshot.play_mode != world::PlayMode::BeforeKickOff) {
        return false;
    }
    const std::array<double, 2> ball{
        context.snapshot.ball.position_m[0],
        context.snapshot.ball.position_m[1],
    };
    const std::array<double, 2> self{
        context.snapshot.self.position_m[0],
        context.snapshot.self.position_m[1],
    };
    return math::planar_dist(self, ball) < field_geometry::kCenterCircleRadiusM;
}

HighLevelCommand make_walk_in_place_command(BehaviorContext& context) {
    WalkCommand walk_cmd;
    walk_cmd.target_2d_m = {
        context.snapshot.self.position_m[0],
        context.snapshot.self.position_m[1],
    };
    walk_cmd.target_absolute = true;
    walk_cmd.role_id = current_role_from_blackboard(context.blackboard);
    return walk_cmd;
}

HighLevelCommand make_kickoff_hold_command(BehaviorContext& context) {
    // The two baseline defenders hold their line for the opening seconds of
    // open play (after TheirKickOff) so they don't release up the pitch the
    // instant the ball is live. role_id is read from the blackboard that
    // compute_formation populated one task earlier in the sequence.
    return make_walk_in_place_command(context);
}

HighLevelCommand make_before_kickoff_hold_command(BehaviorContext& context) {
    // Step in place at the beam pose so the kicker/relay stay ready inside the
    // center circle until the server flips to OurKickOff. Moving the kicker off
    // the center spot during BeforeKickOff would not just waste motion — the
    // server's legality check may relocate them mid-wait.
    return make_walk_in_place_command(context);
}

HighLevelCommand make_set_play_command(BehaviorContext& context) {
    compute_formation(context);
    if (context.snapshot.play_mode == world::PlayMode::TheirKickOff) {
        reset_role_behavior_state();
        return make_walk_in_place_command(context);
    }

    const int role_id = current_role_from_blackboard(context.blackboard);

    if (role_id == RoleManager::ROLE_AP &&
        context.snapshot.play_mode != world::PlayMode::OurGoalKick) {
        // OurGoalKick is excluded because the GK is the designated taker there
        // and an approaching AP would crowd the keeper's clearance.
        auto selected = select_role_behavior(
            context.snapshot, context.blackboard, context.role_manager);
        if (selected.has_value()) {
            return *selected;
        }
    }

    if (role_id == RoleManager::ROLE_GK) {
        auto selected = select_role_behavior(
            context.snapshot, context.blackboard, context.role_manager);
        if (selected.has_value()) {
            return *selected;
        }
    }

    return walk_to_current_role_position(context);
}

NodeResult make_role_behavior_command(BehaviorContext& context) {
    const int role_id = current_role_from_blackboard(context.blackboard);
    auto selected = select_role_behavior(
        context.snapshot, context.blackboard, context.role_manager);
    if (!selected.has_value()) {
        return NodeResult::failure();
    }

    if (auto* walk = std::get_if<WalkCommand>(&*selected); walk != nullptr) {
        walk->role_id = role_id;
    }
    return NodeResult::with_command(*selected);
}

HighLevelCommand make_neutral_command(BehaviorContext&) {
    return NeutralCommand{};
}

NodePtr make_top_level_tree() {
    return bt::fallback<BehaviorContext>({
        bt::sequence<BehaviorContext>({
            bt::condition<BehaviorContext>(has_teammates),
            bt::fallback<BehaviorContext>({
                bt::sequence<BehaviorContext>({
                    bt::condition<BehaviorContext>(needs_beam),
                    bt::command<BehaviorContext>(make_beam_command),
                }),
                bt::sequence<BehaviorContext>({
                    bt::condition<BehaviorContext>([](const BehaviorContext& context) { return is_fallen(context.snapshot); }),
                    bt::command<BehaviorContext>(make_get_up_command),
                }),
                bt::sequence<BehaviorContext>({
                    bt::condition<BehaviorContext>(is_set_play_mode),
                    bt::command<BehaviorContext>(make_set_play_command),
                }),
                bt::sequence<BehaviorContext>({
                    bt::condition<BehaviorContext>(is_before_kickoff_hold),
                    bt::task<BehaviorContext>(compute_formation),
                    bt::command<BehaviorContext>(make_before_kickoff_hold_command),
                }),
                bt::sequence<BehaviorContext>({
                    bt::condition<BehaviorContext>(is_kickoff_hold_active),
                    bt::task<BehaviorContext>(compute_formation),
                    bt::command<BehaviorContext>(make_kickoff_hold_command),
                }),
                bt::sequence<BehaviorContext>({
                    bt::task<BehaviorContext>(compute_formation),
                    bt::task<BehaviorContext>(make_role_behavior_command),
                }),
            }),
        }),
        bt::command<BehaviorContext>(make_neutral_command),
    });
}

}  // namespace

HighLevelCommand BehaviorTree::evaluate(
    const world::WorldSnapshot& snapshot,
    Blackboard& blackboard,
    RoleManager& role_manager) const {
    blackboard.clear();
    update_kickoff_hold_state(snapshot);

    BehaviorContext context{snapshot, blackboard, role_manager};
    static const NodePtr top_level_tree = make_top_level_tree();
    const auto result = top_level_tree->tick(context);
    return result.command.value_or(NeutralCommand{});
}

}  // namespace decision
