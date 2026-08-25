// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 The Hong Kong University of Science and Technology (Guangzhou), Humanoid Computing & Learning Lab.

#include "src/world/world_state.h"

#include "src/math/math_utils.h"
#include "src/server/server_constants.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_map>

namespace world {

namespace {

Vec3 deg_sph2cart(const server::PolarObservation& polar) {
    const double azimuth_rad = math::deg_to_rad(polar.azimuth_deg);
    const double elevation_rad = math::deg_to_rad(polar.elevation_deg);
    const double cos_elevation = std::cos(elevation_rad);
    return {polar.distance_m * cos_elevation * std::cos(azimuth_rad),
            polar.distance_m * cos_elevation * std::sin(azimuth_rad),
            polar.distance_m * std::sin(elevation_rad)};
}

Vec3 camera_position_world(
    const SelfState& self_state,
    const robot::T1RobotModel& robot_model,
    double head_yaw_deg,
    double head_pitch_deg) {
    const Vec3 camera_pos_torso = robot_model.camera_position_torso(head_yaw_deg, head_pitch_deg);
    return math::vec3_add(self_state.position_m, math::rotate_vec_by_quaternion(camera_pos_torso, self_state.orientation_wxyz));
}

Vec3 camera_local_to_world(
    const SelfState& self_state,
    const Vec3& camera_local,
    double head_yaw_deg,
    double head_pitch_deg) {
    const Vec3 head_rotated = math::rotate_z(math::rotate_y(camera_local, head_pitch_deg), head_yaw_deg);
    return math::vec3_add(self_state.position_m, math::rotate_vec_by_quaternion(head_rotated, self_state.orientation_wxyz));
}

bool looks_like_teammate(const std::string& team_name, const std::string& own_team_name) {
    return !team_name.empty() && team_name == own_team_name;
}

bool has_close_position(
    const std::vector<PlayerObservation>& players,
    const Vec3& position_m,
    double distance_threshold_m) {
    const double threshold_sq = distance_threshold_m * distance_threshold_m;
    for (const auto& player : players) {
        if (!player.seen) {
            continue;
        }
        if (math::sq_dist2({player.position_m[0], player.position_m[1]},
                           {position_m[0], position_m[1]}) < threshold_sq) {
            return true;
        }
    }
    return false;
}

double joint_or(const SelfState& self, const char* name) {
    const auto it = self.joint_positions_deg.find(name);
    return it != self.joint_positions_deg.end() ? it->second : 0.0;
}

// Where the server deterministically drops the ball for a restart, in the
// canonical frame (own goal at -x). Used to seed the ball estimate when we are
// blind and have no teammate report: snapping to a generic field center is wrong
// for a goal kick, where the server places the ball at the kicking team's
// goalie-area center (soccer_referee.py goal_kick -> goalie_area.center()). On
// OUR goal kick that spot is ~25 m behind midfield next to our own goal, so a
// center seed would turn the keeper/kicker away from the ball and it would never
// re-acquire. Restarts without a fixed drop point (kickoff is at center anyway;
// throw-in / free kick depend on where play stopped) fall through to center,
// which the fresh-last-known branch usually covers in practice. Corner kicks
// never reach here — their locked anchor keeps ball.visible = true upstream.
Vec3 set_play_ball_anchor(PlayMode mode) {
    constexpr double kGoalieCenterX =
        server_constants::kFieldHalfLengthM - server_constants::kGoalieAreaDepthM * 0.5;
    switch (mode) {
        case PlayMode::OurGoalKick:
            return {-kGoalieCenterX, 0.0, server_constants::kBallRadiusM};
        case PlayMode::TheirGoalKick:
            return {kGoalieCenterX, 0.0, server_constants::kBallRadiusM};
        default:
            return {0.0, 0.0, server_constants::kBallRadiusM};
    }
}

}  // namespace

WorldState::WorldState(std::string team_name, int player_number, int max_players_per_team) {
    snapshot_.team_name = std::move(team_name);
    snapshot_.player_number = player_number;
    snapshot_.teammates.resize(static_cast<std::size_t>(max_players_per_team));
    snapshot_.opponents.resize(static_cast<std::size_t>(max_players_per_team));
    for (std::size_t i = 0; i < snapshot_.teammates.size(); ++i) {
        snapshot_.teammates[i].player_number = static_cast<int>(i) + 1;
        snapshot_.teammates[i].is_teammate = true;
        snapshot_.teammates[i].last_seen_time = -1.0;
        snapshot_.teammates[i].fallen = false;
        snapshot_.opponents[i].player_number = static_cast<int>(i) + 1;
        snapshot_.opponents[i].is_teammate = false;
        snapshot_.opponents[i].last_seen_time = -1.0;
        snapshot_.opponents[i].fallen = false;
    }
    last_opponent_position_m_.assign(snapshot_.opponents.size(), Vec3{0.0, 0.0, 0.0});
    last_opponent_time_.assign(snapshot_.opponents.size(), -1.0);
}

void WorldState::update_from_perception(
    const server::PerceptionFrame& frame,
    const robot::T1RobotModel& robot_model,
    bool normalized_is_left_team) {
    snapshot_.last_server_time = snapshot_.server_time;
    snapshot_.server_time = frame.server_time;

    if (frame.game_state.has_value()) {
        const auto& gs = *frame.game_state;
        if (gs.team_left.has_value() && gs.team_left.value() == snapshot_.team_name) {
            snapshot_.is_left_team = true;
        } else if (gs.team_right.has_value() && gs.team_right.value() == snapshot_.team_name) {
            snapshot_.is_left_team = false;
        } else if (!snapshot_.is_left_team.has_value()) {
            snapshot_.is_left_team = normalized_is_left_team;
        }

        const bool side = snapshot_.is_left_team.value_or(normalized_is_left_team);
        const PlayMode prev_play_mode = snapshot_.play_mode;
        snapshot_.play_mode = play_mode_from_token(gs.play_mode, side);
        snapshot_.play_mode_group = world::play_mode_group(snapshot_.play_mode, side);
        if (snapshot_.play_mode != prev_play_mode) {
            // Server resets the ball on every mode change (kickoff, set play,
            // goal, etc.), so the next detection may be 25-30m away from the
            // last accepted position. Clear the motion-consistency reference
            // so the first valid tick in the new mode isn't rejected as a
            // phantom teleport. Also drop the last-known POSITION so the
            // set_team_comm_snapshot blind fallback cannot reuse a pre-teleport
            // spot as a "recent" observation in the new mode. (The fallback gates
            // on the time reset to -1 here, so this is belt-and-suspenders, but it
            // keeps the "a mode change invalidates the last sighting" invariant
            // explicit for any future reader.)
            last_known_ball_time_ = -1.0;
            last_known_ball_position_m_ = {0.0, 0.0, 0.0};
            // Ball was teleported by the server (kickoff/set play/goal), so the
            // constant-velocity track is meaningless now; drop it and let the
            // next real detection re-initialize.
            ball_kalman_.reset();
            // The referee may teleport defenders out of the ball-exclusion circle
            // on a set play, so clear the opponent gate too and re-seed from the
            // next detection instead of rejecting the jump as a phantom.
            std::fill(last_opponent_time_.begin(), last_opponent_time_.end(), -1.0);
            // Enter corner-probe so we can disambiguate which of the four
            // corners the ball sits at. The server mode only tells us the
            // kicker's side; the top/bottom corner has to be observed.
            if (snapshot_.play_mode == PlayMode::OurCornerKick ||
                snapshot_.play_mode == PlayMode::TheirCornerKick) {
                corner_probe_state_ = CornerProbeState::Probing;
                corner_probe_start_time_ = frame.server_time;
            } else {
                corner_probe_state_ = CornerProbeState::Idle;
            }
        }
        if (should_reset_beam(snapshot_.play_mode)) {
            snapshot_.has_beamed = false;
        }
    } else if (!snapshot_.is_left_team.has_value()) {
        snapshot_.is_left_team = normalized_is_left_team;
    }

    SelfState& self = snapshot_.self;
    BallState& ball = snapshot_.ball;

    // Server omits lin_vel; finite-difference position into the body frame
    // and clip teleports (post-beam, re-localize) to 0 so the policy never
    // sees a multi-meter-per-second spike.
    const Vec3 prev_position = self.position_m;
    if (frame.position.has_value()) {
        self.position_m = frame.position->xyz_m;
    }
    if (frame.orientation.has_value()) {
        self.orientation_wxyz = frame.orientation->wxyz;
    }

    constexpr double kLinVelMaxSpeedMps = 10.0;
    {
        // Skip the first tick: last_server_time is 0, so dt would equal the
        // absolute server time (often seconds) and the estimate would be huge.
        const bool first_tick = snapshot_.last_server_time <= 0.0;
        const double dt = frame.server_time - snapshot_.last_server_time;
        if (!first_tick && dt > 1.0e-6) {
            const Vec3 delta_w = math::vec3_sub(self.position_m, prev_position);
            const Vec3 lin_vel_w = math::vec3_scale(delta_w, 1.0 / dt);
            if (math::norm3(lin_vel_w) <= kLinVelMaxSpeedMps) {
                const Vec3 lin_vel_b = math::rotate_vec_by_quaternion(
                    lin_vel_w, math::quaternion_conjugate(self.orientation_wxyz));
                self.lin_vel_b = lin_vel_b;
            } else {
                self.lin_vel_b = {0.0, 0.0, 0.0};
            }
        } else {
            self.lin_vel_b = {0.0, 0.0, 0.0};
        }
    }
    if (frame.gyro.has_value()) {
        self.gyro_deg_s = frame.gyro->rot_deg_s;
    }
    if (frame.accel.has_value()) {
        self.accel_mps2 = frame.accel->accel_mps2;
    }

    self.joint_positions_deg.clear();
    self.joint_velocities_deg_s.clear();
    for (const auto& joint : frame.joints) {
        const std::string readable_name = normalize_joint_name(joint.name);
        self.joint_positions_deg[readable_name] = joint.ax_deg;
        self.joint_velocities_deg_s[readable_name] = joint.vx_deg_s;
    }

    ball.visible = false;
    ball.velocity_valid = false;
    ball.velocity_mps = {0.0, 0.0, 0.0};

    // Advance the ball Kalman filter one tick (constant-velocity predict) before
    // folding in this tick's detection. dt matches the self-velocity finite diff.
    {
        const double dt = frame.server_time - snapshot_.last_server_time;
        if (snapshot_.last_server_time > 0.0 && dt > 1.0e-6) {
            ball_kalman_.predict(dt);
        }
    }

    constexpr double kFieldHalfLengthM = server_constants::kFieldHalfLengthM;
    constexpr double kFieldHalfWidthM = server_constants::kFieldHalfWidthM;
    if (frame.vision.ball.has_value()) {
        const double head_yaw = joint_or(self, "Head_yaw");
        const double head_pitch = joint_or(self, "Head_pitch");
        const Vec3 ball_camera_local = deg_sph2cart(frame.vision.ball->polar);
        const Vec3 camera_world = camera_position_world(self, robot_model, head_yaw, head_pitch);
        const Vec3 camera_relative_world =
            camera_local_to_world(self, ball_camera_local, head_yaw, head_pitch);
        const Vec3 proposed_ball = math::vec3_add(
            camera_world, math::vec3_sub(camera_relative_world, self.position_m));

        // Reject phantom detections (corner-flag markers, far landmarks,
        // perception noise) using field bounds + motion consistency. The
        // self-to-ball distance cap was removed because it was exactly large
        // enough to reject legitimate corner kicks (~33m corner-to-center
        // distance on a 7v7 field), and the field-bounds check already
        // filters out anything that isn't on or near the pitch.
        // The vision-related thresholds below mirror rcssservermj defaults;
        // see src/server/server_constants.h for the source mapping.
        constexpr double kBallOutsideMarginM = 3.0;
        const bool within_field = std::abs(proposed_ball[0]) <= kFieldHalfLengthM + kBallOutsideMarginM &&
                                  std::abs(proposed_ball[1]) <= kFieldHalfWidthM + kBallOutsideMarginM;

        // Motion consistency: if we accepted a ball recently, a new detection
        // that is more than a few meters from it is a phantom (corner flag,
        // landmark, or noise). The real ball moves <2m per tick; 12m covers
        // ~2s of motion at top speed. Skip when we have no recent reference
        // (initial frames, after long occlusions, or when comm refreshes the
        // estimate in set_team_comm_snapshot).
        constexpr double kRecentBallS = 2.0;
        const double age_s = frame.server_time - last_known_ball_time_;
        const bool have_recent = last_known_ball_time_ > 0.0 && age_s <= kRecentBallS;
        const bool motion_ok = !have_recent ||
            math::norm3(math::vec3_sub(proposed_ball, last_known_ball_position_m_)) <= server_constants::kBallMaxTeleportM;

        if (within_field && motion_ok) {
            // Feed the RAW accepted detection to the Kalman filter, then publish
            // the smoothed estimate. Measurement noise scales with the self->ball
            // range (far detections are noisier, so trusted less).
            const double self_to_ball =
                math::norm3(math::vec3_sub(proposed_ball, self.position_m));
            ball_kalman_.update(proposed_ball, self_to_ball, frame.server_time);
            ball.position_m = ball_kalman_.position();
            ball.visible = true;
            // The motion-consistency reference must stay the RAW observation so
            // the teleport gate keeps comparing like with like.
            last_known_ball_position_m_ = proposed_ball;
            last_known_ball_time_ = frame.server_time;
        }
    }

    // Corner-kick anchor with a short observation window. The server mode
    // only names the kicker's side, so the top/bottom corner has to be
    // discovered. We enter "Probing" on mode change and try to lock the
    // anchor onto the actual ball within kCornerProbeTimeoutS; if we
    // never see the ball we fall back to the y-based heuristic so the
    // kicker / defenders still have *some* position to act on. Once
    // Locked, the anchor is only used when vision drops the ball AND
    // we have no recent evidence that the ball has moved off-corner
    // (otherwise the ball was kicked and we should not snap it back).
    constexpr double kCornerProbeTimeoutS = 1.0;
    constexpr double kCornerAnchorCornerRadiusM = 2.0;
    constexpr double kCornerAnchorReleaseM = 2.0;
    constexpr double kCornerAnchorRecentS = 2.0;

    if (corner_probe_state_ == CornerProbeState::Probing) {
        if (ball.visible) {
            const double dx = std::abs(ball.position_m[0]) - kFieldHalfLengthM;
            const double dy = std::abs(ball.position_m[1]) - kFieldHalfWidthM;
            if (std::abs(dx) <= kCornerAnchorCornerRadiusM &&
                std::abs(dy) <= kCornerAnchorCornerRadiusM) {
                // Saw the ball in a corner — lock the anchor at the
                // actual observed position, not the snapped one.
                corner_anchor_position_m_ = ball.position_m;
                corner_probe_state_ = CornerProbeState::Locked;
            }
        }
        if (corner_probe_state_ == CornerProbeState::Probing &&
            frame.server_time - corner_probe_start_time_ >= kCornerProbeTimeoutS) {
            // Probe timed out. Fall back to the y-based heuristic so we
            // still anchor to *a* corner rather than leaving the ball
            // invisible for the rest of the set play.
            // play_mode is already canonical (own goal at -x): OurCornerKick
            // always means the opponent goal line (+x) and TheirCornerKick our
            // own goal line (-x) for BOTH teams. The previous is_left gate was
            // spurious double-handling that inverted the anchor for the right
            // team, so derive the end purely from the canonical play mode.
            const bool ball_on_plus_x = snapshot_.play_mode == PlayMode::OurCornerKick;
            const double x = ball_on_plus_x ? kFieldHalfLengthM : -kFieldHalfLengthM;
            // last_known_ball_time_ was reset to -1 on the mode change that
            // started this probe, so a positive value means we actually
            // observed the ball during the probe and last_known_ball_position_m_
            // is a trustworthy side cue. Otherwise the y-side is unknown and we
            // must NOT read a position carried over from a previous play.
            const bool observed_this_probe = last_known_ball_time_ > 0.0;
            const double seed_y =
                observed_this_probe ? last_known_ball_position_m_[1] : 0.0;
            const double y = seed_y < 0.0 ? -kFieldHalfWidthM : kFieldHalfWidthM;
            corner_anchor_position_m_ = {x, y, server_constants::kBallRadiusM};
            corner_probe_state_ = CornerProbeState::Locked;
        }
    }

    if (corner_probe_state_ == CornerProbeState::Locked && !ball.visible) {
        const double dist_from_anchor = math::norm3(math::vec3_sub(
            last_known_ball_position_m_, corner_anchor_position_m_));
        const bool have_recent = last_known_ball_time_ > 0.0 &&
                                 (frame.server_time - last_known_ball_time_) <= kCornerAnchorRecentS;
        const bool anchor_stale = have_recent && dist_from_anchor >= kCornerAnchorReleaseM;
        if (!anchor_stale) {
            ball.position_m = corner_anchor_position_m_;
            ball.visible = true;
            // Do NOT write the synthesized anchor back into the
            // motion-consistency reference (last_known_ball_position_m_/time_):
            // that reference must reflect real observations only. Feeding the
            // anchor back would pin dist_from_anchor at 0 (the anchor could
            // never go stale) and freeze the teleport-gate reference at the
            // corner, so a ball kicked away and re-detected far off would be
            // rejected forever as a phantom for the rest of the set play.
        }
    }

    // Publish the Kalman velocity estimate (canonical frame). velocity_valid is
    // gated conservatively (>=2 updates, low covariance, fresh) so the tuned kick
    // and formation logic only ever see a velocity the filter is confident in --
    // never the noise-amplified single-tick finite difference that made a raw
    // velocity signal unusable.
    if (ball_kalman_.has_track()) {
        ball.velocity_mps = ball_kalman_.velocity();
        ball.velocity_valid = ball_kalman_.velocity_confident(frame.server_time);
    }

    for (auto& player : snapshot_.teammates) {
        player.seen = false;
        // Keep `fallen`; vision doesn't update it, and comm records may not
        // arrive every tick. Wiping to false could silently elect a fallen
        // teammate as AP (select_ap excludes only fallen==true).
    }
    for (auto& player : snapshot_.opponents) {
        player.seen = false;
        player.fallen = false;
    }

    if (snapshot_.player_number > 0 &&
        static_cast<std::size_t>(snapshot_.player_number) <= snapshot_.teammates.size()) {
        auto& self_player = snapshot_.teammates[static_cast<std::size_t>(snapshot_.player_number - 1)];
        self_player.player_number = snapshot_.player_number;
        self_player.is_teammate = true;
        self_player.seen = true;
        self_player.position_m = self.position_m;
        self_player.last_seen_time = snapshot_.server_time;
        self_player.fallen = self.position_m[2] < kFallenHeightThresholdM;
    }

    const double head_yaw = joint_or(self, "Head_yaw");
    const double head_pitch = joint_or(self, "Head_pitch");
    const Vec3 camera_world = camera_position_world(self, robot_model, head_yaw, head_pitch);

    for (const auto& detected_player : frame.vision.players) {
        if (detected_player.player_number <= 0) {
            continue;
        }
        Vec3 centroid{0.0, 0.0, 0.0};
        double marker_count = 0.0;
        for (const auto& marker : detected_player.markers) {
            const Vec3 marker_local = deg_sph2cart(marker.polar);
            const Vec3 marker_world_offset =
                camera_local_to_world(self, marker_local, head_yaw, head_pitch);
            centroid = math::vec3_add(centroid, math::vec3_sub(marker_world_offset, self.position_m));
            marker_count += 1.0;
        }
        if (marker_count <= 0.0) {
            continue;
        }
        centroid = math::vec3_divide(centroid, marker_count);
        const Vec3 player_world = math::vec3_add(camera_world, centroid);

        const bool is_teammate = looks_like_teammate(detected_player.team_name, snapshot_.team_name);
        auto& bucket = is_teammate ? snapshot_.teammates : snapshot_.opponents;
        const std::size_t idx = static_cast<std::size_t>(detected_player.player_number - 1);
        if (idx >= bucket.size()) {
            continue;
        }
        bucket[idx].player_number = detected_player.player_number;
        if (is_teammate) {
            // Teammate vision positions are corroborated by exact self-reports
            // over team comm, so leave them raw (no per-slot gate state kept).
            bucket[idx].seen = true;
            bucket[idx].position_m = player_world;
            bucket[idx].last_seen_time = snapshot_.server_time;
            continue;
        }

        // Opponent: reject phantom teleports (false positives / class confusion)
        // and lightly smooth accepted detections. Opponents move slowly
        // (~1-2 m/s), so a multi-metre jump within a second is not real motion.
        constexpr double kOpponentRecentS = 1.0;
        constexpr double kOpponentMaxTeleportM = 4.0;
        constexpr double kOpponentEmaAlpha = 0.5;
        const double opp_age = snapshot_.server_time - last_opponent_time_[idx];
        const bool opp_recent =
            last_opponent_time_[idx] > 0.0 && opp_age <= kOpponentRecentS;
        if (opp_recent &&
            math::norm3(math::vec3_sub(player_world, last_opponent_position_m_[idx])) >
                kOpponentMaxTeleportM) {
            continue;  // drop phantom; leave the slot unseen this tick
        }
        const Vec3 filtered = opp_recent
            ? math::vec3_add(
                  math::vec3_scale(player_world, kOpponentEmaAlpha),
                  math::vec3_scale(last_opponent_position_m_[idx], 1.0 - kOpponentEmaAlpha))
            : player_world;
        bucket[idx].seen = true;
        bucket[idx].position_m = filtered;
        bucket[idx].last_seen_time = snapshot_.server_time;
        last_opponent_position_m_[idx] = filtered;
        last_opponent_time_[idx] = snapshot_.server_time;
    }
}

void WorldState::set_team_comm_snapshot(const comm::TeamCommSnapshot& comm_snapshot) {
    snapshot_.team_comm_snapshot = comm_snapshot;
    snapshot_.shared_opponents.clear();
    // `comm_role` is per-tick and reset to -1 here (record loop below sets
    // it back). `fallen` is preserved — see update_from_perception.
    for (auto& teammate : snapshot_.teammates) {
        teammate.comm_role = -1;
    }

    for (const auto& record : snapshot_.team_comm_snapshot.records) {
        if (record.sender_player_number <= 0 ||
            static_cast<std::size_t>(record.sender_player_number) > snapshot_.teammates.size() ||
            record.sender_player_number == snapshot_.player_number) {
            continue;
        }
        auto& mate = snapshot_.teammates[static_cast<std::size_t>(record.sender_player_number - 1)];
        mate.fallen = record.fallen;
        mate.comm_role = record.current_role;

        if (!mate.seen) {
            mate.position_m = {record.self_x_m, record.self_y_m, 0.5};
            mate.last_seen_time = snapshot_.server_time;
        }
    }

    if (!snapshot_.ball.visible) {
        // Blind receiver: fuse ALL fresh teammate ball reports by inverse
        // variance instead of trusting the single latest one. Each report's
        // weight comes from the sender's own->ball range (sigma grows ~linearly
        // with distance; the range is derived from the self and ball fields
        // already in the packet, so no protocol change is needed) plus a
        // staleness penalty. An anchor guard drops reports far from the freshest
        // one: for a slow/near-stationary ball all reports cluster and average
        // (variance reduction, benchmark: ~40-57% RMSE cut), but for a fast ball
        // the stale, displaced reports fall away and the estimate collapses to
        // the freshest -- so fusion never snaps the ball onto a stale position
        // during fast play.
        constexpr double kAnchorGuardM = 1.5;
        constexpr double kMeasNoisePerMeter = 0.012;
        constexpr double kStalenessPerCycle = 0.05;
        constexpr double kMinMeasurementRangeM = 0.5;

        // Freshest cycle sets the age reference for the staleness penalty.
        int freshest_cycle = 0;
        bool any_ball = false;
        for (const auto& record : snapshot_.team_comm_snapshot.records) {
            if (!record.ball_seen || record.sender_player_number == snapshot_.player_number) continue;
            if (!any_ball || record.server_cycle > freshest_cycle) freshest_cycle = record.server_cycle;
            any_ball = true;
        }

        auto weight_of = [&](const comm::TeamCommRecord& record) {
            const double dx = record.ball_x_m - record.self_x_m;
            const double dy = record.ball_y_m - record.self_y_m;
            const double sender_range = std::sqrt(dx * dx + dy * dy);
            const double age_cycles = static_cast<double>(freshest_cycle - record.server_cycle);
            const double vision_sigma = kMeasNoisePerMeter * std::max(sender_range, kMinMeasurementRangeM);
            const double stale_sigma = kStalenessPerCycle * age_cycles;
            const double variance = vision_sigma * vision_sigma + stale_sigma * stale_sigma;
            return variance > 1.0e-9 ? 1.0 / variance : 0.0;
        };

        // Anchor the outlier guard on the MOST RELIABLE report (highest weight =
        // closest, freshest sender), not merely the newest: a single far,
        // misclassified "ball" has low weight and so can't become the anchor and
        // drag the estimate.
        const comm::TeamCommRecord* anchor_rec = nullptr;
        double best_weight = -1.0;
        for (const auto& record : snapshot_.team_comm_snapshot.records) {
            if (!record.ball_seen || record.sender_player_number == snapshot_.player_number) continue;
            const double w = weight_of(record);
            if (w > best_weight) { best_weight = w; anchor_rec = &record; }
        }
        // Track whether comm fusion produced a usable position this tick. If it
        // did not, the fallback below refreshes ball.position_m instead of leaving
        // a stale value in place (see the !fused_from_comm branch).
        bool fused_from_comm = false;
        if (anchor_rec) {
            const Vec3 anchor{anchor_rec->ball_x_m, anchor_rec->ball_y_m, server_constants::kBallRadiusM};
            Vec3 weighted_sum{0.0, 0.0, 0.0};
            double weight_total = 0.0;
            for (const auto& record : snapshot_.team_comm_snapshot.records) {
                if (!record.ball_seen) continue;
                if (record.sender_player_number == snapshot_.player_number) continue;
                const Vec3 ball_xy{record.ball_x_m, record.ball_y_m, server_constants::kBallRadiusM};
                if (math::norm3(math::vec3_sub(ball_xy, anchor)) >= kAnchorGuardM) {
                    continue;  // stale/displaced or misclassified outlier
                }
                const double weight = weight_of(record);
                weighted_sum = math::vec3_add(weighted_sum, math::vec3_scale(ball_xy, weight));
                weight_total += weight;
            }
            if (weight_total > 0.0) {
                snapshot_.ball.position_m = math::vec3_scale(weighted_sum, 1.0 / weight_total);
                snapshot_.ball.position_m[2] = server_constants::kBallRadiusM;
                snapshot_.ball.velocity_valid = false;
                snapshot_.ball.velocity_mps = {0.0, 0.0, 0.0};
                // Keep the motion-consistency reference fresh so the next
                // perception tick can reject teleports from the fused estimate.
                last_known_ball_position_m_ = snapshot_.ball.position_m;
                last_known_ball_time_ = snapshot_.server_time;
                fused_from_comm = true;
            }
        }

        if (!fused_from_comm) {
            // Self vision is blind AND comm fusion produced nothing usable:
            // anchor_rec == nullptr (no teammate reported the ball this tick), or
            // every report failed the anchor guard so weight_total stayed 0.
            // Leaving ball.position_m untouched would let a stale value survive --
            // most damagingly the pre-teleport position carried across a set-play
            // mode change, where the server moves the ball 20-30 m. The GK target,
            // the RL ball observation, and the formation anchor all read
            // ball.position_m, so that stale value makes the keeper face and hold a
            // ball that is no longer there and never re-acquire the real one.
            // Refresh instead: prefer a real observation while it is still fresh
            // (the ball may already have been kicked and moved off the drop point),
            // otherwise seed the server's deterministic restart drop point for the
            // current mode. That point is mode-specific: on OUR goal kick the ball
            // sits at our goalie-area center next to our own goal, ~25 m from
            // midfield, so a plain field-center seed would point the keeper the
            // wrong way -- exactly the "keeper loses the ball on our goal kick"
            // failure. Keep ball.visible = false so downstream treats this as a
            // best-effort guess, not a sighting. The 2 s window matches kRecentBallS
            // / kCornerAnchorRecentS; last_known_ball_time_ is reset to -1 on every
            // mode change, so right after a teleport this deliberately uses the
            // mode drop point rather than the stale last-known spot. Corner kicks
            // never reach here: their Locked anchor keeps ball.visible = true.
            constexpr double kFallbackFreshnessS = 2.0;
            const bool have_recent_last =
                last_known_ball_time_ > 0.0 &&
                (snapshot_.server_time - last_known_ball_time_) <= kFallbackFreshnessS;
            snapshot_.ball.position_m = have_recent_last
                ? last_known_ball_position_m_
                : set_play_ball_anchor(snapshot_.play_mode);
            snapshot_.ball.velocity_valid = false;
            snapshot_.ball.velocity_mps = {0.0, 0.0, 0.0};
        }
    }

    for (const auto& record : snapshot_.team_comm_snapshot.records) {
        if (!record.opponent_seen || record.sender_player_number == snapshot_.player_number) {
            continue;
        }

        const Vec3 position_m{record.opponent_x_m, record.opponent_y_m, 0.5};
        if (has_close_position(snapshot_.opponents, position_m, 1.0) ||
            has_close_position(snapshot_.shared_opponents, position_m, 1.0)) {
            continue;
        }

        PlayerObservation opponent;
        opponent.player_number = 0;
        opponent.is_teammate = false;
        opponent.seen = true;
        opponent.last_seen_time = snapshot_.server_time;
        opponent.position_m = position_m;
        snapshot_.shared_opponents.push_back(opponent);
    }
}

std::string WorldState::normalize_joint_name(const std::string& name) {
    static const std::unordered_map<std::string, std::string> kServerToReadable = {
        {"q_hj1", "Head_yaw"},
        {"q_hj2", "Head_pitch"},
        {"q_laj1", "Left_Shoulder_Pitch"},
        {"q_laj2", "Left_Shoulder_Roll"},
        {"q_laj3", "Left_Elbow_Pitch"},
        {"q_laj4", "Left_Elbow_Yaw"},
        {"q_raj1", "Right_Shoulder_Pitch"},
        {"q_raj2", "Right_Shoulder_Roll"},
        {"q_raj3", "Right_Elbow_Pitch"},
        {"q_raj4", "Right_Elbow_Yaw"},
        {"q_tj1", "Waist"},
        {"q_llj1", "Left_Hip_Pitch"},
        {"q_llj2", "Left_Hip_Roll"},
        {"q_llj3", "Left_Hip_Yaw"},
        {"q_llj4", "Left_Knee_Pitch"},
        {"q_llj5", "Left_Ankle_Pitch"},
        {"q_llj6", "Left_Ankle_Roll"},
        {"q_rlj1", "Right_Hip_Pitch"},
        {"q_rlj2", "Right_Hip_Roll"},
        {"q_rlj3", "Right_Hip_Yaw"},
        {"q_rlj4", "Right_Knee_Pitch"},
        {"q_rlj5", "Right_Ankle_Pitch"},
        {"q_rlj6", "Right_Ankle_Roll"},
    };
    const auto it = kServerToReadable.find(name);
    if (it != kServerToReadable.end()) {
        return it->second;
    }
    return name;
}

}  // namespace world
