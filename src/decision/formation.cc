// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 The Hong Kong University of Science and Technology (Guangzhou), Humanoid Computing & Learning Lab.

// Formation layouts for the 7v7 humanoid team.
//
// Public entry points produce 7-role formations
// ([GK, CBL, CBR, CDM, CBM, ST, AP]):
//   * Set-play shapes (kickoff / goal kick / corner / generic) are hardcoded
//     coordinate tables; tune them directly for your team's preferences.
//   * make_open_play_shape computes open-play targets dynamically from the
//     ball position.
//
// --- Base-version scope ---
// make_open_play_shape is intentionally simple so other teams can fork and
// customize. It is NOT aware of:
//   * game phase (no Defense / Balanced / Attack mode switch — see git
//     history for the removed ~110-line ShapeMode-driven version),
//   * opponent positions,
//   * ball velocity.
// FormationContext exposes only ball_position_m, play_mode, field_length_m,
// and field_width_m. If you need any of the above signals, add them back to
// FormationContext and consume them inside make_open_play_shape.

#include "src/decision/formation.h"
#include "src/decision/field_geometry.h"
#include "src/decision/role_manager.h"
#include "src/math/math_utils.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace decision {

namespace {

using field_geometry::Position2;

constexpr double kRoleBallClearanceM = 1.2;
constexpr double kStBallClearanceM = 2.4;            // ST needs more room to turn / receive a pass

// CB pair flanks the ball at kCbDistFromBallM along the ball→own_goal axis,
// split kCbLateralOffsetM perpendicular to each side.
constexpr double kCbDistFromBallM = 3.0;
constexpr double kCbLateralOffsetM = 4.0;

// CBM and CDM on the ball→own_goal axis, between the ball and the GK.
constexpr double kCbmDistFromBallM = 4.0;
constexpr double kCdmDistFromBallM = 7.0;

// ST sits this far ahead of the ball on the ball→opponent-goal axis, with a
// small lateral offset to avoid standing exactly on the ball's line.
constexpr double kStAttackDepthM = 3.0;
constexpr double kStLateralOffsetM = 1.3;

constexpr std::size_t kRoleCbl = static_cast<std::size_t>(RoleManager::ROLE_CBL);
constexpr std::size_t kRoleCbr = static_cast<std::size_t>(RoleManager::ROLE_CBR);
constexpr std::size_t kRoleCdm = static_cast<std::size_t>(RoleManager::ROLE_CDM);
constexpr std::size_t kRoleCbm = static_cast<std::size_t>(RoleManager::ROLE_CBM);
constexpr std::size_t kRoleSt = static_cast<std::size_t>(RoleManager::ROLE_ST);

Position2 clamp_to_field(
    Position2 pos,
    double half_length_m,
    double half_width_m,
    double margin_m = field_geometry::kFormationFieldMarginM) {
    pos[0] = std::clamp(pos[0], -half_length_m + margin_m, half_length_m - margin_m);
    pos[1] = std::clamp(pos[1], -half_width_m + margin_m, half_width_m - margin_m);
    return pos;
}

// ST relay target: ball + a lateral offset perpendicular to the ball→their_goal
// axis. Used by every our-kick set-play shape so the relay sits beside the
// kicker (not directly behind it), giving ST a clean one-touch reception.
Position2 st_relay_position(const Position2& ball,
                            const Position2& their_goal,
                            double half_length_m,
                            double half_width_m,
                            double distance_m = field_geometry::kSetPlayRelayDistanceM) {
    const Position2 goal_dir = math::vec2_unit_or(math::vec2_sub(their_goal, ball), {1.0, 0.0});
    const Position2 lateral = math::perpendicular_left(goal_dir);
    return clamp_to_field(math::vec2_add(ball, math::vec2_scale(lateral, distance_m)),
                          half_length_m, half_width_m);
}

// Defensive AP target: distance_m from the ball along the ball→own_goal axis.
// clamp_to_field pulls the target back inside if the ball is on the byline.
Position2 ap_position_on_defensive_line(
    const Position2& ball,
    const Position2& own_goal,
    double distance_m,
    double half_length_m,
    double half_width_m) {
    const Position2 dir = math::vec2_unit_or(math::vec2_sub(own_goal, ball), {1.0, 0.0});
    return clamp_to_field(math::vec2_add(ball, math::vec2_scale(dir, distance_m)),
                          half_length_m, half_width_m);
}

Position2 keep_clear_of_ball(
    Position2 pos,
    const Position2& ball,
    const Position2& fallback_dir,
    double min_dist_m,
    double half_length_m,
    double half_width_m) {
    const Position2 offset = math::vec2_sub(pos, ball);
    const double d = math::norm2(offset);
    if (d >= min_dist_m) {
        return clamp_to_field(pos, half_length_m, half_width_m);
    }
    const Position2 dir = math::vec2_unit_or(offset, fallback_dir);
    return clamp_to_field(math::vec2_add(ball, math::vec2_scale(dir, min_dist_m)), half_length_m, half_width_m);
}

void avoid_goal_mouth(Position2& pos, double half_length_m) {
    const bool near_their_goal = pos[0] > half_length_m - 1.8;
    const bool near_our_goal = pos[0] < -half_length_m + 1.8;
    if ((near_their_goal || near_our_goal) && std::abs(pos[1]) < field_geometry::kGoalHalfWidthM + 0.5) {
        const double sign = pos[1] < 0.0 ? -1.0 : 1.0;
        pos[1] = sign * (field_geometry::kGoalHalfWidthM + 0.8);
    }
}

// Project a CDM target onto the ball→own_goal axis at a fixed distance from
// own_goal. Used by set-play shapes that need CDM colinear with the ball.
Position2 cdm_on_ball_goal_axis(
    const Position2& ball,
    const Position2& own_goal,
    double distance_m,
    double half_length_m,
    double half_width_m) {
    const Position2 dir = math::vec2_unit_or(math::vec2_sub(ball, own_goal), {1.0, 0.0});
    return clamp_to_field(
        math::vec2_add(own_goal, math::vec2_scale(dir, distance_m)),
        half_length_m, half_width_m);
}

Position2 point_from_anchor_toward(
    const Position2& anchor,
    const Position2& target,
    const Position2& fallback_dir,
    double distance_m,
    double half_length_m,
    double half_width_m) {
    const Position2 dir = math::vec2_unit_or(math::vec2_sub(target, anchor), fallback_dir);
    return clamp_to_field(
        math::vec2_add(anchor, math::vec2_scale(dir, distance_m)),
        half_length_m, half_width_m);
}

Formation::RolePositions make_open_play_shape(const FormationContext& ctx) {
    const double half_length = ctx.field_length_m * 0.5;
    const double half_width = ctx.field_width_m * 0.5;
    const Position2 own_goal{-half_length, 0.0};
    const Position2 their_goal{half_length, 0.0};
    const Position2 ball = clamp_to_field(ctx.ball_position_m, half_length, half_width);

    // GK on the semicircle from own goal toward ball.
    const Position2 goalkeeper = clamp_to_field(
        field_geometry::goalkeeper_semicircle_position(ball, half_length),
        half_length,
        half_width);

    // Goal→ball axis + fallback direction; reused for the rest of the line.
    const Position2 u = math::vec2_unit_or(math::vec2_sub(ball, own_goal), {1.0, 0.0});
    const Position2 gk_to_ball = math::vec2_unit_or(math::vec2_sub(ball, goalkeeper), u);

    // CBL/CBR flank the ball at kCbDistFromBallM along the ball→own_goal axis,
    // one on each perpendicular side.
    const Position2 cb_anchor = point_from_anchor_toward(
        ball, own_goal, u, kCbDistFromBallM, half_length, half_width);
    const Position2 cb_lateral = math::perpendicular_left(u);
    Position2 cbl = clamp_to_field(
        math::vec2_add(cb_anchor, math::vec2_scale(cb_lateral, +kCbLateralOffsetM)),
        half_length, half_width);
    Position2 cbr = clamp_to_field(
        math::vec2_add(cb_anchor, math::vec2_scale(cb_lateral, -kCbLateralOffsetM)),
        half_length, half_width);
    cbl = keep_clear_of_ball(cbl, ball, math::vec2_scale(gk_to_ball, -1.0), kRoleBallClearanceM, half_length, half_width);
    cbr = keep_clear_of_ball(cbr, ball, math::vec2_scale(gk_to_ball, -1.0), kRoleBallClearanceM, half_length, half_width);

    // CBM and CDM on the ball→own_goal axis, between the ball and the GK.
    Position2 cbm = point_from_anchor_toward(
        ball, own_goal, u, kCbmDistFromBallM, half_length, half_width);
    Position2 cdm = point_from_anchor_toward(
        ball, own_goal, u, kCdmDistFromBallM, half_length, half_width);
    cbm = keep_clear_of_ball(cbm, ball, math::vec2_scale(u, -1.0), kRoleBallClearanceM, half_length, half_width);
    cdm = keep_clear_of_ball(cdm, ball, math::vec2_scale(u, -1.0), kRoleBallClearanceM, half_length, half_width);

    // ST sits kStAttackDepthM ahead of the ball on the ball→opponent-goal
    // axis, with a small lateral offset so it isn't on the ball's exact line.
    // The lateral side follows the ball's half of the field (y>0 → up,
    // y<0 → down) so ST mirrors the attack side instead of always fanning to
    // one perpendicular side.
    const Position2 goal_dir = math::vec2_unit_or(math::vec2_sub(their_goal, ball), {1.0, 0.0});
    const Position2 goal_lateral = math::perpendicular_left(goal_dir);
    const double st_side = ball[1] >= 0.0 ? 1.0 : -1.0;
    Position2 st = clamp_to_field(
        math::vec2_add(
            math::vec2_add(ball, math::vec2_scale(goal_dir, kStAttackDepthM)),
            math::vec2_scale(goal_lateral, st_side * kStLateralOffsetM)),
        half_length, half_width);
    st = keep_clear_of_ball(st, ball, math::vec2_scale(goal_lateral, st_side), kStBallClearanceM, half_length, half_width);
    avoid_goal_mouth(st, half_length);

    return {
        goalkeeper,
        clamp_to_field(cbl, half_length, half_width),
        clamp_to_field(cbr, half_length, half_width),
        clamp_to_field(cdm, half_length, half_width),
        clamp_to_field(cbm, half_length, half_width),
        clamp_to_field(st, half_length, half_width),
        ctx.ball_position_m,
    };
}

Formation::RolePositions make_kickoff_shape(
    const FormationContext& ctx,
    bool our_kickoff) {
    const double half_length = ctx.field_length_m * 0.5;
    const double half_width = ctx.field_width_m * 0.5;
    const Position2 ball = ctx.ball_position_m;
    const Position2 their_goal{half_length, 0.0};
    const Position2 st_relay = st_relay_position(ball, their_goal, half_length, half_width);

    Formation::RolePositions positions{{
        field_geometry::goalkeeper_semicircle_position(ball, half_length),
        {-14.0, 5.5},
        {-14.0, -5.5},
        {-15.0, 0.0},
        {-21.0, 1.0},
        st_relay,
        {-10.0, 0.0},
    }};
    if (!our_kickoff) {
        // Defending the kickoff: every field player must stay in our half
        // (x < 0) and outside the center circle (norm > kCenterCircleRadiusM
        // + safety). legalize_set_play_target tightens any remaining slack.
        positions[0] = field_geometry::goalkeeper_semicircle_position(ball, half_length);
        positions[kRoleCbl] = {-17.0, 5.5};
        positions[kRoleCbr] = {-17.0, -5.5};
        positions[kRoleCdm] = {-14.5, 0.0};
        positions[kRoleCbm] = {-10.0, 2.0};
        positions[kRoleSt] = {-8.5, -2.0};
        positions[6] = {-8.0, 0.0};
    }
    for (std::size_t i = 0; i + 1 < positions.size(); ++i) {
        positions[i] = clamp_to_field(positions[i], half_length, half_width);
    }
    return positions;
}

Formation::RolePositions make_goal_kick_shape(
    const FormationContext& ctx,
    bool our_goal_kick) {
    const double half_length = ctx.field_length_m * 0.5;
    const double half_width = ctx.field_width_m * 0.5;
    const Position2 own_goal{-half_length, 0.0};
    const Position2 their_goal{half_length, 0.0};
    const Position2 ball = ctx.ball_position_m;
    const Position2 ap_pos = our_goal_kick
        ? ball
        : ap_position_on_defensive_line(ball, own_goal, 6.0, half_length, half_width);

    Formation::RolePositions positions;
    if (our_goal_kick) {
        const Position2 st_relay = st_relay_position(ball, their_goal, half_length, half_width);
        positions = {{
            field_geometry::goalkeeper_semicircle_position(ball, half_length),
            {-15.0, 6.8},
            {-15.0, -6.8},
            cdm_on_ball_goal_axis(ball, own_goal, 9.5, half_length, half_width),
            {-20.0, 4.0},
            st_relay,
            ball,
        }};
    } else {
        positions = {{
            field_geometry::goalkeeper_semicircle_position(ball, half_length),
            {3.0, 8.0},
            {3.0, -8.0},
            {-10.0, 0.0},  // CDM stays in our half as a counter-attack anchor
            {-2.0, 0.0},  // CBM in front of CDM
            {12.0, 0.0},  // ST
            ap_pos,
        }};
    }
    for (std::size_t i = 0; i + 1 < positions.size(); ++i) {
        positions[i] = clamp_to_field(positions[i], half_length, half_width);
    }
    return positions;
}

Formation::RolePositions make_corner_shape(
    const FormationContext& ctx,
    bool our_corner) {
    const double half_length = ctx.field_length_m * 0.5;
    const double half_width = ctx.field_width_m * 0.5;
    const double side = ctx.ball_position_m[1] >= 0.0 ? 1.0 : -1.0;
    const Position2 own_goal{-half_length, 0.0};
    const Position2 their_goal{half_length, 0.0};
    const Position2 ball = ctx.ball_position_m;
    const Position2 ap_pos = our_corner
        ? ball
        : ap_position_on_defensive_line(ball, own_goal, 6.0, half_length, half_width);

    Formation::RolePositions positions;
    if (our_corner) {
        const Position2 st_relay = st_relay_position(ball, their_goal, half_length, half_width);
        positions = {{
            field_geometry::goalkeeper_semicircle_position(ball, half_length),
            {side > 0 ? 12.0 : 14.0, side > 0 ? 5.0 : 0.0},
            {side > 0 ? 14.0 : 12.0, side > 0 ? 0.0 : -5.0},
            {-8.0, 0.0},  // CDM stays in our half as a counter-attack anchor
            {5.0, side * 3.0},  // CBM pushes into the opponent half
            st_relay,
            ap_pos,
        }};
    } else {
        positions = {{
            field_geometry::goalkeeper_semicircle_position(ball, half_length),
            {-half_length + 3.2, side * 1.8},
            {-half_length + 3.4, -side * 2.1},
            cdm_on_ball_goal_axis(ball, own_goal, 5.76, half_length, half_width),
            {-half_length + 9.5, -side * 4.8},
            {-6.0, -side * 6.2},
            ap_pos,
        }};
    }
    for (std::size_t i = 0; i + 1 < positions.size(); ++i) {
        positions[i] = clamp_to_field(positions[i], half_length, half_width);
    }
    return positions;
}

Formation::RolePositions make_generic_set_play_shape(
    const FormationContext& ctx,
    bool our_kick) {
    const double half_length = ctx.field_length_m * 0.5;
    const double half_width = ctx.field_width_m * 0.5;
    const Position2 own_goal{-half_length, 0.0};
    const Position2 their_goal{half_length, 0.0};
    const Position2 ball = clamp_to_field(ctx.ball_position_m, half_length, half_width);
    const Position2 ap_pos = our_kick
        ? ball
        : ap_position_on_defensive_line(ball, own_goal, 6.0, half_length, half_width);
    const Position2 u = math::vec2_unit_or(math::vec2_sub(ball, own_goal), {1.0, 0.0});
    const Position2 p = math::perpendicular_left(u);
    const Position2 goal_dir = math::vec2_unit_or(math::vec2_sub(their_goal, ball), {1.0, 0.0});
    const double side = ball[1] <= 0.0 ? 1.0 : -1.0;

    Formation::RolePositions positions;
    if (our_kick) {
        const Position2 st_relay = st_relay_position(ball, their_goal, half_length, half_width);
        positions = {{
            field_geometry::goalkeeper_semicircle_position(ball, half_length),
            math::vec2_add(own_goal, {9.0, 5.5}),
            math::vec2_add(own_goal, {9.0, -5.5}),
            math::vec2_add(ball, math::vec2_scale(u, -7.0)),
            math::vec2_add(math::vec2_add(ball, math::vec2_scale(u, -3.5)), math::vec2_scale(p, side * 5.4)),
            st_relay,
            ap_pos,
        }};
    } else {
        // Server rule: defending team must stay outside kCenterCircleRadiusM
        // (5.5m) from the ball during ThrowIn/CornerKick/FreeKick/
        // DirectFreeKick. Place every field player naturally beyond that
        // radius so legalize_set_play_target is a no-op and the formation
        // shape isn't warped by post-processing.
        const double cb_dist = std::min(std::max(6.0, math::norm2(math::vec2_sub(ball, own_goal)) * 0.42), 18.0);
        const Position2 cb_center = math::vec2_add(own_goal, math::vec2_scale(u, cb_dist));
        const double cb_lateral = 4.0;
        positions = {{
            field_geometry::goalkeeper_semicircle_position(ball, half_length),
            math::vec2_add(cb_center, math::vec2_scale(p, cb_lateral)),
            math::vec2_add(cb_center, math::vec2_scale(p, -cb_lateral)),
            math::vec2_add(ball, math::vec2_scale(u, -6.0)),
            math::vec2_add(math::vec2_add(ball, math::vec2_scale(u, -7.0)), math::vec2_scale(p, side * 4.5)),
            {-5.5, -side * 7.0},
            ap_pos,
        }};
    }

    for (std::size_t i = 0; i + 1 < positions.size(); ++i) {
        positions[i] = clamp_to_field(positions[i], half_length, half_width);
        positions[i] = keep_clear_of_ball(
            positions[i],
            ball,
            i == kRoleSt ? goal_dir : math::vec2_scale(u, -1.0),
            // 5.5m matches the server's exclusion radius for opponent set
            // plays. For our own set plays we still want some standoff so we
            // don't walk into the ball while approaching.
            our_kick ? 3.2 : 5.5,
            half_length,
            half_width);
    }
    return positions;
}

bool is_our_generic_set_play(world::PlayMode play_mode) {
    return play_mode == world::PlayMode::OurThrowIn ||
           play_mode == world::PlayMode::OurFreeKick ||
           play_mode == world::PlayMode::OurDirectFreeKick ||
           play_mode == world::PlayMode::OurOffside ||
           play_mode == world::PlayMode::OurPenaltyKick ||
           play_mode == world::PlayMode::OurPenaltyShoot;
}

bool is_their_generic_set_play(world::PlayMode play_mode) {
    return play_mode == world::PlayMode::TheirThrowIn ||
           play_mode == world::PlayMode::TheirFreeKick ||
           play_mode == world::PlayMode::TheirDirectFreeKick ||
           play_mode == world::PlayMode::TheirOffside ||
           play_mode == world::PlayMode::TheirPenaltyKick ||
           play_mode == world::PlayMode::TheirPenaltyShoot;
}


}  // namespace

Formation::RolePositions Formation::compute(const FormationContext& ctx) const {
    switch (ctx.play_mode) {
    case world::PlayMode::OurKickOff:
        return make_kickoff_shape(ctx, true);
    case world::PlayMode::TheirKickOff:
        return make_kickoff_shape(ctx, false);
    case world::PlayMode::OurGoalKick:
        return make_goal_kick_shape(ctx, true);
    case world::PlayMode::TheirGoalKick:
        return make_goal_kick_shape(ctx, false);
    case world::PlayMode::OurCornerKick:
        return make_corner_shape(ctx, true);
    case world::PlayMode::TheirCornerKick:
        return make_corner_shape(ctx, false);
    default:
        break;
    }

    if (is_our_generic_set_play(ctx.play_mode)) {
        return make_generic_set_play_shape(ctx, true);
    }
    if (is_their_generic_set_play(ctx.play_mode)) {
        return make_generic_set_play_shape(ctx, false);
    }
    return make_open_play_shape(ctx);
}

}  // namespace decision
