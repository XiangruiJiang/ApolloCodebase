// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 The Hong Kong University of Science and Technology (Guangzhou), Humanoid Computing & Learning Lab.

#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cmath>

#include "src/math/math_utils.h"
#include "src/world/play_mode.h"

/// Canonical-field geometry and server-legality helpers, expressed in meters.
namespace decision::field_geometry {

/// Two-dimensional position `(x, y)` in the canonical field frame.
using Position2 = std::array<double, 2>;
/// Beam pose `(x, y, yaw_deg)` in the canonical field frame.
using Pose3 = std::array<double, 3>;

inline constexpr double kBaseFieldLengthM = 30.0;
inline constexpr double kBaseFieldWidthM = 20.0;
inline constexpr double kActualFieldLengthM = 55.0;
inline constexpr double kActualFieldWidthM = 36.0;
inline constexpr double kActualHalfLengthM = kActualFieldLengthM * 0.5;
inline constexpr double kActualHalfWidthM = kActualFieldWidthM * 0.5;
inline constexpr double kCenterCircleRadiusM = 5.5;
inline constexpr double kSetPlaySafetyMarginM = 0.2;
inline constexpr double kGoalieAreaDepthM = 4.0;
inline constexpr double kGoalieAreaWidthM = 7.3;
inline constexpr double kGoalHalfWidthM = 1.83;

// Goalkeeper resting stance: the keeper stands kGkHoldDepthM in front of its
// own goal-line center while the GK is not handling a set play.
inline constexpr double kGkHoldDepthM = 0.5;

// Hard clamp margin that keeps formation slots inside the field. Used by
// the formation shape and the set-play defensive AP target.
inline constexpr double kFormationFieldMarginM = 0.7;

// Earlier boundary-avoidance trigger for the A* walk planner. Larger than
// kFormationFieldMarginM because the planner needs a smooth turn well before
// the actual field edge.
inline constexpr double kWalkPlannerFieldMarginM = 1.5;

inline constexpr std::array<Position2, 7> kBasePlayerPoses{{
    {-14.5, 0.0},
    {-10.0, 3.0},
    {-10.0, -3.0},
    {-6.0, 0.0},
    {-2.0, 4.0},
    {-2.0, -4.0},
    {-0.55, 0.0},
}};

// Defensive (opponent) kickoff standing shape. On an opponent kickoff every one
// of our field players must stay legal for the server's placement check, i.e.
//   x < 0            (our own half; x > 0 is penalized/relocated), and
//   hypot(x, y) > R  (outside the R = kCenterCircleRadiusM = 5.5 m center circle).
// Players 6 & 7 form the deep guard pair kDefensiveKickoffBaselineDepthM off our
// own goal line (split +/-kDefensiveKickoffBaselineSpreadYM about the goal
// center). The other outfielders (2, 3, 4, 5) press just outside the center
// circle so the team is not bunched deep in its own half. All poses below are in
// ACTUAL-field meters (goal line at -kActualHalfLengthM) and keep >= ~1.3 m of
// margin on both legality limits.
inline constexpr double kDefensiveKickoffBaselineDepthM = 4.0;
inline constexpr double kDefensiveKickoffBaselineSpreadYM = 3.0;
inline constexpr Position2 kDefensiveKickoffPressCenterPose{-7.0, 0.0};  // striker press, faces the ball
inline constexpr Position2 kDefensiveKickoffPressWingPose{-6.5, 5.0};    // wing press (y mirrored per side)
inline constexpr Position2 kDefensiveKickoffScreenPose{-12.0, 0.0};      // central screen ahead of the deep pair

inline constexpr Position2 to_actual_field(
    const Position2& base_position,
    double actual_field_length_m = kActualFieldLengthM,
    double actual_field_width_m = kActualFieldWidthM) {
    return {
        base_position[0] * actual_field_length_m / kBaseFieldLengthM,
        base_position[1] * actual_field_width_m / kBaseFieldWidthM,
    };
}

inline constexpr Position2 actual_their_goal_center_target() {
    return {kActualHalfLengthM, 0.0};
}

// AP simplified "push" handoff. When the AP is within kPushBallEngageDistanceM
// of the ball it switches from walking-to-ball to walking past the ball toward
// the opponent goal; the walk target is offset kPushPastBallM past the ball
// along the ball->goal direction so the robot's forward motion pushes the ball
// along.
inline constexpr double kPushBallEngageDistanceM = 1.0;
inline constexpr double kPushPastBallM = 1.0;
inline constexpr double kSetPlayRelayBallSpeedMps = 0.8;
inline constexpr double kSetPlayRelayDistanceM = 1.2;

// Kickoff kicker (player 7) sits this far behind the ball (own-goal-ward) and
// this far to its right (kicker's right, i.e. -y in the canonical frame where
// +x is the kicker's heading). Faces the relay (player 6) at beam time so the
// push direction toward the relay slot reads as a forward push, not a turn.
inline constexpr double kKickoffKickerBehindM = 1.0;
inline constexpr double kKickoffKickerLateralM = 0.5;

// "Close enough to the target" handoff radius shared by the walk planner and
// the obstacle-avoidance skip path. Below this distance, planning degenerates
// and the robot should just head straight for the target.
inline constexpr double kNearTargetM = 0.3;

/// Returns the default formation position for a 1-based player number.
inline Position2 player_fallback_position(int player_number) {
    const std::size_t idx = player_number > 0 && player_number <= static_cast<int>(kBasePlayerPoses.size())
                                ? static_cast<std::size_t>(player_number - 1)
                                : 0U;
    return to_actual_field(kBasePlayerPoses[idx]);
}

// Kickoff relay slot: lateral offset from the kicker anchor along the ball→goal
// axis. The kicker anchor reuses kBasePlayerPoses[6] so the relay lines up
// with the formation's ST slot at kickoff (perpendicular_left of the heading,
// +kSetPlayRelayDistanceM).
inline Position2 kickoff_relay_slot() {
    const Position2 kicker_anchor = to_actual_field(kBasePlayerPoses[6]);
    const Position2 goal_dir = math::vec2_unit_or(
        {kActualHalfLengthM - kicker_anchor[0], 0.0 - kicker_anchor[1]}, {1.0, 0.0});
    const Position2 lateral = math::perpendicular_left(goal_dir);
    return {
        kicker_anchor[0] + lateral[0] * kSetPlayRelayDistanceM,
        kicker_anchor[1] + lateral[1] * kSetPlayRelayDistanceM,
    };
}

/// Returns the default beam pose for a 1-based player number.
inline Pose3 player_beam_pose(int player_number) {
    // Player 7 (kicker) stands behind-right of the ball at beam time,
    // oriented along the diagonal push heading; player 6 (relay) takes the
    // perpendicular slot. Everyone else uses a fixed fallback pose so the
    // open-play formation takes over cleanly once the ball is in play.
    if (player_number == 7) {
        // Kicker faces 45° at beam time — matches the diagonal push heading
        // it will travel along, so the walk to ball reads as a forward push
        // and not a 45° turn.
        const Position2 kicker{-kKickoffKickerBehindM, -kKickoffKickerLateralM};
        return {kicker[0], kicker[1], 45.0};
    }
    if (player_number == 6) {
        const Position2 relay = kickoff_relay_slot();
        return {relay[0], relay[1], 0.0};
    }
    const Position2 position = player_fallback_position(player_number);
    return {position[0], position[1], 0.0};
}

/// Returns a legal defensive-kickoff beam pose for a 1-based player number.
inline Pose3 player_defensive_kickoff_beam_pose(int player_number) {
    // Players 6 and 7 drop to the deep guard pair 4 m off our own goal line; 2/3
    // press the wings, 5 presses through the middle, and 4 screens centrally in
    // front of the deep pair. Everyone else (the GK) keeps its default goal-mouth
    // beam pose. See the shape constants above for the legality margins (own half
    // + outside the center circle).
    const double baseline_x = -kActualHalfLengthM + kDefensiveKickoffBaselineDepthM;
    switch (player_number) {
        case 6:
            return {baseline_x, kDefensiveKickoffBaselineSpreadYM, 0.0};
        case 7:
            return {baseline_x, -kDefensiveKickoffBaselineSpreadYM, 0.0};
        case 2:
            return {kDefensiveKickoffPressWingPose[0], kDefensiveKickoffPressWingPose[1], 0.0};
        case 3:
            return {kDefensiveKickoffPressWingPose[0], -kDefensiveKickoffPressWingPose[1], 0.0};
        case 4:
            return {kDefensiveKickoffScreenPose[0], kDefensiveKickoffScreenPose[1], 0.0};
        case 5:
            return {kDefensiveKickoffPressCenterPose[0], kDefensiveKickoffPressCenterPose[1], 0.0};
        default:
            break;
    }
    const Position2 position = player_fallback_position(player_number);
    return {position[0], position[1], 0.0};
}

/// Places the goalkeeper on a goal-centered semicircle facing the ball.
inline Position2 goalkeeper_semicircle_position(
    const Position2& ball_position,
    double half_length_m = kActualHalfLengthM,
    double radius_m = kGoalieAreaDepthM) {
    // Minimum standoff from the goal line so the keeper never collapses onto
    // it (e.g. ball behind the goal line during corners would otherwise produce
    // a target with dx=0).
    constexpr double kMinDepthM = 0.5;
    const Position2 own_goal{-half_length_m, 0.0};
    const double dx = std::max(kMinDepthM, ball_position[0] - own_goal[0]);
    const double dy = ball_position[1] - own_goal[1];
    const double norm = math::norm2({dx, dy});

    Position2 target;
    if (norm < 1e-6) {
        target = {own_goal[0] + radius_m, own_goal[1]};
    } else {
        target = {
            own_goal[0] + radius_m * dx / norm,
            own_goal[1] + radius_m * dy / norm,
        };
    }

    // Keep the keeper inside the goal mouth in the y axis and at least
    // kMinDepthM in front of the goal line.
    target[0] = std::max(target[0], own_goal[0] + kMinDepthM);
    target[1] = std::clamp(target[1], -kGoalHalfWidthM, kGoalHalfWidthM);
    return target;
}

inline double squared_norm(const Position2& position) {
    return position[0] * position[0] + position[1] * position[1];
}

inline bool is_in_their_goalie_area(const Position2& position) {
    const double min_x = kActualHalfLengthM - kGoalieAreaDepthM;
    const double half_width_m = kGoalieAreaWidthM * 0.5;
    return position[0] >= min_x && position[0] <= kActualHalfLengthM &&
           std::abs(position[1]) <= half_width_m;
}

inline bool is_in_our_goalie_area(const Position2& position) {
    const double max_x = -kActualHalfLengthM + kGoalieAreaDepthM;
    const double half_width_m = kGoalieAreaWidthM * 0.5;
    return position[0] <= max_x && position[0] >= -kActualHalfLengthM &&
           std::abs(position[1]) <= half_width_m;
}

inline Position2 project_outside_center_circle_in_our_half(
    const Position2& position,
    double radius_m = kCenterCircleRadiusM,
    double safety_margin_m = kSetPlaySafetyMarginM) {
    const double legal_radius = radius_m + safety_margin_m;
    Position2 projected = position;
    projected[0] = std::min(projected[0], -safety_margin_m);

    const double projected_sq_norm = squared_norm(projected);
    if (projected_sq_norm < legal_radius * legal_radius) {
        const double required_abs_y =
            std::sqrt(legal_radius * legal_radius - projected[0] * projected[0]);
        const double y_sign = projected[1] < 0.0 ? -1.0 : 1.0;
        projected[1] = y_sign * required_abs_y;
    }

    return projected;
}

inline Position2 project_outside_ball_exclusion_circle(
    const Position2& position,
    const Position2& ball_position,
    double radius_m = kCenterCircleRadiusM,
    double safety_margin_m = kSetPlaySafetyMarginM) {
    const double legal_radius = radius_m + safety_margin_m;
    Position2 offset{
        position[0] - ball_position[0],
        position[1] - ball_position[1],
    };
    const double offset_sq_norm = squared_norm(offset);
    if (offset_sq_norm >= legal_radius * legal_radius) {
        return position;
    }

    if (offset_sq_norm < 1e-12) {
        return {ball_position[0] - legal_radius, ball_position[1]};
    }

    const double scale = legal_radius / std::sqrt(offset_sq_norm);
    return {
        ball_position[0] + offset[0] * scale,
        ball_position[1] + offset[1] * scale,
    };
}

/// Projects a formation target onto the legal region for the current set play.
inline Position2 legalize_set_play_target(
    const Position2& target_position,
    const Position2& ball_position,
    world::PlayMode play_mode) {
    if (play_mode == world::PlayMode::TheirKickOff ||
        play_mode == world::PlayMode::OurKickOff) {
        // The server enforces "stay in own half + outside center circle" for
        // both kickoff variants (the kicker / AP path is handled separately).
        return project_outside_center_circle_in_our_half(target_position);
    }
    if (play_mode == world::PlayMode::TheirGoalKick &&
        is_in_their_goalie_area(target_position)) {
        return {
            kActualHalfLengthM - kGoalieAreaDepthM - kSetPlaySafetyMarginM,
            target_position[1],
        };
    }
    if (play_mode == world::PlayMode::OurGoalKick &&
        is_in_our_goalie_area(target_position)) {
        return {
            -kActualHalfLengthM + kGoalieAreaDepthM + kSetPlaySafetyMarginM,
            target_position[1],
        };
    }
    // The server only teleports the DEFENDING team out of the ball-exclusion
    // circle, and only for opponent throw-in / corner / free / direct-free
    // restarts (_check_placement_for_free_kick_* in soccer_referee.py). Our own
    // restarts and offside/penalty are never penalized this way, so don't
    // displace our designed formation slots for them (the set-play shapes
    // already keep our players clear of the ball via keep_clear_of_ball).
    if (play_mode == world::PlayMode::TheirThrowIn ||
        play_mode == world::PlayMode::TheirCornerKick ||
        play_mode == world::PlayMode::TheirFreeKick ||
        play_mode == world::PlayMode::TheirDirectFreeKick) {
        return project_outside_ball_exclusion_circle(target_position, ball_position);
    }
    return target_position;
}

}  // namespace decision::field_geometry
