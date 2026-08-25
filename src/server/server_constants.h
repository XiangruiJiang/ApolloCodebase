// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 The Hong Kong University of Science and Technology (Guangzhou), Humanoid Computing & Learning Lab.

#pragma once

// Client-side mirror of the rcssservermj simulation parameters the client
// must agree with the server on.
//
// The historical filename was "vision_constants"; this header collects the
// universe of constants the world / comm / decision layers all need to keep
// in lockstep with rcssservermj (see rcssservermj/src/rcsssmj/sim/...) —
// not only vision defaults. Lives under src/server/ specifically because
// that module has no project-internal dependencies, so every other module
// can include it without creating a cycle (decision → world → server is the
// only viable direction).
//
// What the server actually owns (and the client therefore does NOT
// re-simulate): dist_sigma, angle_sigma, fn_rate, fp_rate,
// confusion_rate, max_number_of_false_positives,
// decimal_position_precision, send_unique_class_names,
// vision_interval, check_occlusion. These surface as noise / dropouts /
// landmarks inside (See ...) and are trusted verbatim.

/// Client-side constants that must stay consistent with RCSSServerMJ.
namespace server_constants {

// Ball radius, and therefore the ball-center height when it is resting on the
// pitch. Mirrors the server's SoccerBall._radius (== rcssservermj
// resources/environments/soccer/world.xml ball geom size) and the training
// asset (robocup_rl_mjlab ball.xml). Used as the z default/fallback whenever
// the ball height is not available from a fresh own-vision detection (Kalman
// init and the team-comm relay path, which carries x/y only). Keeping this in
// sync with the server prevents the policy from seeing a biased rel_pos_b[2].
inline constexpr double kBallRadiusM = 0.11;

// Client policy: if a new ball detection is farther than this from the
// last accepted ball within kRecentBallS seconds, it is treated as a
// phantom (corner flag, landmark, or perception noise) and rejected.
// 12 m covers ~2 s of motion at top speed.
inline constexpr double kBallMaxTeleportM = 12.0;

// Field half-extents used by the world layer for ball-in-field and corner
// anchor checks. The world layer cannot include decision/field_geometry.h
// (decision depends on world, not the reverse), so these mirror
// field_geometry::kActualHalfLengthM / kActualHalfWidthM (FIFA 55 x 36 m
// field) and MUST be kept in sync with them.
inline constexpr double kFieldHalfLengthM = 27.5;
inline constexpr double kFieldHalfWidthM = 18.0;

// Goalie-area depth (x extent from the goal line). Mirrors
// field_geometry::kGoalieAreaDepthM and the server's FIFA7vs7 goalie_area_dim[0]
// (soccer_fields.py). Used by the world layer to seed the ball estimate at the
// server's deterministic goal-kick drop point (the goalie-area center). MUST be
// kept in sync with both.
inline constexpr double kGoalieAreaDepthM = 4.0;

}  // namespace server_constants
