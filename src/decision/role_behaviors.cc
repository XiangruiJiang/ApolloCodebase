// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 The Hong Kong University of Science and Technology (Guangzhou), Humanoid Computing & Learning Lab.

#include "src/decision/role_behaviors.h"

#include "src/decision/behavior_nodes.h"
#include "src/decision/walk_planner.h"
#include "src/decision/field_geometry.h"
#include "src/decision/role_manager.h"
#include "src/math/math_utils.h"
#include "src/world/frame_normalizer.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>

namespace decision {

namespace {

constexpr double kWalkMaxSpeedM = 3.0;
constexpr double kWalkBrakeDecelMps2 = 1.0;
constexpr double kWalkStopRadiusM = 0.15;
constexpr double kWalkHeadingSlowStartDeg = 35.0;
constexpr double kWalkHeadingStopDeg = 95.0;
// At or below this distance to the target a ball-facing player strafes (holds
// ball-facing, no heading-error speed penalty) to fine-tune its slot; beyond it
// the player turns to face its travel direction and runs forward at full speed.
constexpr double kStrafeMaxDistM = 2.5;
constexpr double kDribbleSetupDistanceM = 0.6;
constexpr double kDribbleSetupToleranceM = 0.25;
constexpr double kDribbleSideDistanceM = 0.8;
constexpr double kDribbleSideClearanceM = 0.55;
constexpr double kDribbleSideStepBehindThresholdM = 0.1;
constexpr double kDribbleMaxLateralOffsetM = 0.4;
constexpr double kDribbleMaxAheadM = 0.15;

bool is_our_set_play(const world::WorldSnapshot& snapshot) {
    return snapshot.play_mode_group == world::PlayModeGroup::OurKick;
}

std::array<double, 2> role_position_from_blackboard(const Blackboard& blackboard) {
    if (!blackboard.exists(Blackboard::kKeyRolePos)) {
        return {0.0, 0.0};
    }
    return blackboard.get<std::array<double, 2>>(Blackboard::kKeyRolePos);
}

WalkCommand make_walk_command(const std::array<double, 2>& target_position_m) {
    WalkCommand command;
    command.target_2d_m = target_position_m;
    command.target_absolute = true;
    command.orientation_deg = std::nullopt;
    command.orientation_absolute = true;
    return command;
}

double walk_speed_command(double distance_m, double heading_error_abs_deg) {
    // Physical braking curve: v = sqrt(2 * a * (d - stop_radius)). Continuous
    // through zero at d = kWalkStopRadiusM and feasible to track for a robot
    // with peak deceleration a ~= kWalkBrakeDecelMps2.
    if (distance_m <= kWalkStopRadiusM) {
        return 0.0;
    }
    const double effective_dist = distance_m - kWalkStopRadiusM;
    const double brake = std::sqrt(2.0 * kWalkBrakeDecelMps2 * effective_dist);
    double speed = std::min(kWalkMaxSpeedM, brake);

    if (heading_error_abs_deg > kWalkHeadingSlowStartDeg) {
        const double t = std::clamp(
            (kWalkHeadingStopDeg - heading_error_abs_deg) /
                (kWalkHeadingStopDeg - kWalkHeadingSlowStartDeg),
            0.1,
            1.0);
        speed *= t;
    }

    return speed;
}

std::optional<double> orientation_to_point_from_self(
    const world::WorldSnapshot& snapshot,
    const std::array<double, 2>& target) {
    const double dx = target[0] - snapshot.self.position_m[0];
    const double dy = target[1] - snapshot.self.position_m[1];
    if (dx * dx + dy * dy <= 1e-6) {
        return std::nullopt;
    }
    return math::vector_angle_deg({dx, dy});
}

std::optional<double> orientation_to_ball_from_self(const world::WorldSnapshot& snapshot) {
    return orientation_to_point_from_self(
        snapshot,
        {snapshot.ball.position_m[0], snapshot.ball.position_m[1]});
}

WalkCommand make_walk_command_avoiding(
    const std::array<double, 2>& target_position_m,
    const world::WorldSnapshot& snapshot,
    std::optional<double> opponent_x_threshold = std::nullopt,
    bool avoid_field_boundaries = true,
    bool orient_to_ball = true,
    std::optional<int> role_id = std::nullopt,
    bool suppress_heading_slowdown = false,
    bool avoid_obstacles = true) {
    const std::array<double, 2> self{snapshot.self.position_m[0], snapshot.self.position_m[1]};
    const double dist = math::planar_dist(self, target_position_m);

    if (dist < field_geometry::kNearTargetM) {
        WalkCommand command = make_walk_command(target_position_m);
        command.orientation_deg = orient_to_ball
            ? orientation_to_ball_from_self(snapshot)
            : std::nullopt;
        command.orientation_absolute = true;
        command.role_id = role_id;
        return command;
    }

    const auto plan = plan_walk(
        self,
        target_position_m,
        snapshot,
        snapshot.player_number,
        opponent_x_threshold,
        avoid_field_boundaries,
        avoid_obstacles);
    const double heading_rad = math::deg_to_rad(plan.heading_deg);
    const double self_yaw_deg =
        world::FrameNormalizer::yaw_deg_from_quaternion_wxyz(snapshot.self.orientation_wxyz);
    const double rel_heading = heading_rad - math::deg_to_rad(self_yaw_deg);
    const double heading_error_abs_deg = std::abs(math::normalize_deg(plan.heading_deg - self_yaw_deg));

    // Orientation vs. travel policy. Facing the ball keeps a player ready to
    // react, but holding ball-facing while travelling elsewhere means it can only
    // strafe/backpedal — and the heading-error speed penalty throttles that to
    // ~10% (~0.3 m/s). So split by travel distance:
    //   - strafe mode (close to the target, or a forced sideways mover such as
    //     the keeper): face the ball and drop the heading penalty, so short
    //     lateral adjustments run at the walk's natural lateral cap;
    //   - sprint mode (far from the target): face the travel direction and keep
    //     the penalty, so the player turns and runs forward at full speed, then
    //     re-faces the ball as it arrives (dist falls into strafe range).
    // A player travelling toward the ball (the AP) has travel ~= ball direction,
    // so the two modes coincide and its behavior is unchanged. The walk runner
    // still clamps |vy| in every mode, so strafing stays within its safe cap.
    const bool strafe = suppress_heading_slowdown ||
                        (orient_to_ball && dist <= kStrafeMaxDistM);
    const double speed =
        walk_speed_command(dist, strafe ? 0.0 : heading_error_abs_deg);

    WalkCommand command;
    command.target_2d_m = {speed * std::cos(rel_heading), speed * std::sin(rel_heading)};
    command.target_absolute = false;
    command.orientation_deg = (strafe && orient_to_ball)
        ? orientation_to_ball_from_self(snapshot).value_or(plan.heading_deg)
        : plan.heading_deg;
    command.orientation_absolute = true;
    command.role_id = role_id;
    return command;
}

bool match_role(const Blackboard& blackboard, int role_id) {
    return current_role_from_blackboard(blackboard) == role_id;
}

struct APDecisionContext {
    const world::WorldSnapshot& snapshot;
    APState& state;
    std::array<double, 2> ball{0.0, 0.0};
    std::array<double, 2> self{0.0, 0.0};
    double ball_distance{0.0};
};

using APNodePtr = bt::NodePtr<APDecisionContext>;

struct GKDecisionContext {
    const world::WorldSnapshot& snapshot;
    std::array<double, 2> ball{0.0, 0.0};
    std::array<double, 2> self{0.0, 0.0};
    double ball_distance{0.0};
};

using GKNodePtr = bt::NodePtr<GKDecisionContext>;

WalkCommand make_dribble_command(
    APDecisionContext& context,
    double absolute_direction_deg) {
    const double direction_rad = math::deg_to_rad(absolute_direction_deg);
    const std::array<double, 2> direction{
        std::cos(direction_rad),
        std::sin(direction_rad),
    };
    const std::array<double, 2> perpendicular{-direction[1], direction[0]};
    const std::array<double, 2> setup_target{
        context.ball[0] - direction[0] * kDribbleSetupDistanceM,
        context.ball[1] - direction[1] * kDribbleSetupDistanceM,
    };
    const std::array<double, 2> push_target{
        context.ball[0] + direction[0] * field_geometry::kPushPastBallM,
        context.ball[1] + direction[1] * field_geometry::kPushPastBallM,
    };
    const std::array<double, 2> self_from_ball{
        context.self[0] - context.ball[0],
        context.self[1] - context.ball[1],
    };
    const double along_direction =
        self_from_ball[0] * direction[0] + self_from_ball[1] * direction[1];
    const double signed_lateral_offset =
        self_from_ball[0] * perpendicular[0] + self_from_ball[1] * perpendicular[1];
    const double lateral_offset = std::abs(signed_lateral_offset);
    const bool needs_side_step =
        along_direction > -kDribbleSideStepBehindThresholdM &&
        lateral_offset < kDribbleSideClearanceM;
    std::array<double, 2> approach_target = setup_target;
    if (needs_side_step) {
        const double side_sign = signed_lateral_offset < 0.0 ? -1.0 : 1.0;
        approach_target = {
            context.ball[0] + perpendicular[0] * side_sign * kDribbleSideDistanceM,
            context.ball[1] + perpendicular[1] * side_sign * kDribbleSideDistanceM,
        };
    }
    const bool setup_ready =
        math::planar_dist(context.self, setup_target) <= kDribbleSetupToleranceM;

    if (!context.state.dribble_ready && setup_ready) {
        context.state.dribble_ready = true;
    }

    const bool push_position_valid =
        along_direction <= kDribbleMaxAheadM &&
        lateral_offset <= kDribbleMaxLateralOffsetM &&
        context.ball_distance <= field_geometry::kPushBallEngageDistanceM;
    if (context.state.dribble_ready && !push_position_valid) {
        context.state.dribble_ready = false;
    }

    if (!context.state.dribble_ready) {
        WalkCommand approach_command = make_walk_command_avoiding(
            approach_target, context.snapshot, std::nullopt, true, true,
            RoleManager::ROLE_AP, true, false);
        approach_command.orientation_deg = absolute_direction_deg;
        approach_command.orientation_absolute = true;
        return approach_command;
    }

    return make_walk_command_avoiding(
        push_target, context.snapshot, std::nullopt, true, true,
        RoleManager::ROLE_AP, false, false);
}

WalkCommand make_ap_push_ball_to_goal(APDecisionContext& context) {
    // During our-kick set plays push diagonally toward the relay teammate's
    // side (45° off the ball→goal axis) instead of straight at the opponent
    // goal — the relay takes the ball cleanly without the AP chasing its own
    // pass (double-touch). Outside our-kick (PlayOn, their-kick) keep aiming
    // at the goal.
    std::array<double, 2> goal_direction;
    if (context.snapshot.play_mode_group == world::PlayModeGroup::OurKick) {
        const std::array<double, 2> their_goal = field_geometry::actual_their_goal_center_target();
        const std::array<double, 2> goal_dir = math::vec2_unit_or(
            math::vec2_sub(their_goal, context.ball), {1.0, 0.0});
        // vector_angle_deg is atan2, so the relay distance cancels — rotating
        // goal_dir by 45° gives the diagonal push heading directly.
        goal_direction = math::rotate_2d(goal_dir, 45.0);
    } else {
        const std::array<double, 2> their_goal = field_geometry::actual_their_goal_center_target();
        goal_direction = math::vec2_sub(their_goal, context.ball);
    }
    const double absolute_direction_deg = math::norm2(goal_direction) > 1e-6
        ? math::vector_angle_deg(goal_direction)
        : 0.0;
    return make_dribble_command(context, absolute_direction_deg);
}

bool is_gk_our_goal_kick(const GKDecisionContext& context) {
    return context.snapshot.play_mode == world::PlayMode::OurGoalKick;
}

WalkCommand make_gk_walk_to_ball(GKDecisionContext& context) {
    // On OurGoalKick the keeper walks straight at the ball.
    return make_walk_command_avoiding(
        context.ball, context.snapshot, std::nullopt, true, true,
        RoleManager::ROLE_GK);
}

HighLevelCommand make_gk_hold_position(GKDecisionContext& /*context*/) {
    // Stay parked at the keeper's hold point (kGkHoldDepthM in front of our
    // goal line, on the centerline).
    const std::array<double, 2> target{
        -field_geometry::kActualHalfLengthM + field_geometry::kGkHoldDepthM,
        0.0,
    };
    WalkCommand command;
    command.target_2d_m = target;
    command.target_absolute = true;
    command.orientation_deg = std::nullopt;
    command.orientation_absolute = true;
    command.role_id = RoleManager::ROLE_GK;
    return command;
}

const APBehavior& ap_behavior_instance() {
    static const APBehavior instance;
    return instance;
}

const SimpleRoleBehavior& cbm_behavior_instance() {
    static const SimpleRoleBehavior instance{RoleManager::ROLE_CBM, false};
    return instance;
}

const SimpleRoleBehavior& st_behavior_instance() {
    static const SimpleRoleBehavior instance{RoleManager::ROLE_ST, false};
    return instance;
}

const SimpleRoleBehavior& cbl_behavior_instance() {
    static const SimpleRoleBehavior instance{RoleManager::ROLE_CBL, true};
    return instance;
}

const SimpleRoleBehavior& cbr_behavior_instance() {
    static const SimpleRoleBehavior instance{RoleManager::ROLE_CBR, true};
    return instance;
}

const SimpleRoleBehavior& cdm_behavior_instance() {
    static const SimpleRoleBehavior instance{RoleManager::ROLE_CDM, false};
    return instance;
}

const GKBehavior& gk_behavior_instance() {
    static const GKBehavior instance;
    return instance;
}

}  // namespace

int current_role_from_blackboard(const Blackboard& blackboard) {
    return blackboard.exists(Blackboard::kKeyCurrentRole)
        ? blackboard.get<int>(Blackboard::kKeyCurrentRole)
        : -1;
}

bool APBehavior::matches(const Blackboard& blackboard) const {
    return match_role(blackboard, RoleManager::ROLE_AP);
}

HighLevelCommand APBehavior::make_command(
    const world::WorldSnapshot& snapshot,
    const Blackboard& blackboard,
    RoleManager& role_manager) const {
    static const APNodePtr ap_tree = bt::command<APDecisionContext>(make_ap_push_ball_to_goal);

    if (!is_our_set_play(snapshot)) {
        state_.set_play_released = false;
    }

    APDecisionContext context{
        snapshot,
        state_,
        {snapshot.ball.position_m[0], snapshot.ball.position_m[1]},
        {snapshot.self.position_m[0], snapshot.self.position_m[1]},
        0.0};
    context.ball_distance = math::planar_dist(context.ball, context.self);

    if (is_our_set_play(snapshot) && state_.set_play_released) {
        return make_walk_command_avoiding(
            role_position_from_blackboard(blackboard), snapshot, std::nullopt,
            true, true, RoleManager::ROLE_AP);
    }

    const double previous_ball_distance = state_.previous_ball_distance;
    const auto result = ap_tree->tick(context);
    state_.previous_ball_distance = context.ball_distance;
    if (is_our_set_play(snapshot) && state_.dribble_ready &&
        context.ball_distance > field_geometry::kPushBallEngageDistanceM &&
        previous_ball_distance <= field_geometry::kPushBallEngageDistanceM &&
        snapshot.ball.velocity_valid &&
        math::norm2({snapshot.ball.velocity_mps[0], snapshot.ball.velocity_mps[1]}) >=
            field_geometry::kSetPlayRelayBallSpeedMps) {
        state_.set_play_released = true;
        state_.dribble_ready = false;
        role_manager.mark_self_set_play_pushed(snapshot.player_number, snapshot);
    }
    return result.command.value_or(NeutralCommand{});
}

bool SimpleRoleBehavior::matches(const Blackboard& blackboard) const {
    return match_role(blackboard, role_id_);
}

HighLevelCommand SimpleRoleBehavior::make_command(
    const world::WorldSnapshot& snapshot,
    const Blackboard& blackboard) const {
    if (role_id_ == RoleManager::ROLE_ST && is_our_set_play(snapshot)) {
        if (!snapshot.ball.velocity_valid) {
            relay_state_ = {};
        } else {
            const double ball_speed = math::norm2(
                {snapshot.ball.velocity_mps[0], snapshot.ball.velocity_mps[1]});
            if (ball_speed >= field_geometry::kSetPlayRelayBallSpeedMps) {
                APDecisionContext relay_context{
                    snapshot,
                    relay_state_,
                    {snapshot.ball.position_m[0], snapshot.ball.position_m[1]},
                    {snapshot.self.position_m[0], snapshot.self.position_m[1]},
                    0.0};
                relay_context.ball_distance = math::planar_dist(relay_context.ball, relay_context.self);
                return make_dribble_command(relay_context, 0.0);
            }
        }
    } else {
        relay_state_ = {};
    }

    std::optional<double> opponent_x_threshold;
    if (defensive_opponent_clip_) {
        opponent_x_threshold = snapshot.ball.position_m[0] - 1.0;
    }
    return make_walk_command_avoiding(
        role_position_from_blackboard(blackboard), snapshot, opponent_x_threshold);
}

bool GKBehavior::matches(const Blackboard& blackboard) const {
    return match_role(blackboard, RoleManager::ROLE_GK);
}

HighLevelCommand GKBehavior::make_command(
    const world::WorldSnapshot& snapshot,
    const Blackboard& /*blackboard*/) const {
    static const GKNodePtr gk_tree = bt::fallback<GKDecisionContext>({
        bt::sequence<GKDecisionContext>({
            bt::condition<GKDecisionContext>(is_gk_our_goal_kick),
            bt::command<GKDecisionContext>(make_gk_walk_to_ball),
        }),
        bt::command<GKDecisionContext>(make_gk_hold_position),
    });

    GKDecisionContext context{
        snapshot,
        {snapshot.ball.position_m[0], snapshot.ball.position_m[1]},
        {snapshot.self.position_m[0], snapshot.self.position_m[1]},
        0.0};
    context.ball_distance = math::planar_dist(context.ball, context.self);

    const auto result = gk_tree->tick(context);
    return result.command.value_or(NeutralCommand{});
}

std::optional<HighLevelCommand> select_role_behavior(
    const world::WorldSnapshot& snapshot,
    const Blackboard& blackboard,
    RoleManager& role_manager) {
    // AP is the only behavior that needs RoleManager (to latch the set-play
    // push); dispatch it directly and let the other behaviors share the
    // 2-param base interface.
    if (ap_behavior_instance().matches(blackboard)) {
        return ap_behavior_instance().make_command(snapshot, blackboard, role_manager);
    }
    static const std::array<const RoleBehavior*, 6> behaviors{
        &cbm_behavior_instance(),
        &st_behavior_instance(),
        &cbl_behavior_instance(),
        &cbr_behavior_instance(),
        &cdm_behavior_instance(),
        &gk_behavior_instance(),
    };

    for (const auto* behavior : behaviors) {
        if (behavior->matches(blackboard)) {
            return behavior->make_command(snapshot, blackboard);
        }
    }
    return std::nullopt;
}

void reset_role_behavior_state() {
    ap_behavior_instance().reset_state();
    gk_behavior_instance().reset_state();
}

}  // namespace decision
