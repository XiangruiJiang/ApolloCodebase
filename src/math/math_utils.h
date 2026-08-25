// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 The Hong Kong University of Science and Technology (Guangzhou), Humanoid Computing & Learning Lab.

#pragma once

#include <algorithm>
#include <array>
#include <cmath>

/// Dependency-free geometry helpers. Quaternions use `(w, x, y, z)` order.
namespace math {

constexpr double kPi = 3.14159265358979323846;

inline double deg_to_rad(double degrees) {
    return degrees * kPi / 180.0;
}

inline double rad_to_deg(double radians) {
    return radians * 180.0 / kPi;
}

/// Wraps an angle to the half-open range `[-180, 180)` degrees.
inline double normalize_deg(double angle_deg) {
    double value = std::fmod(angle_deg + 180.0, 360.0);
    if (value < 0.0) {
        value += 360.0;
    }
    return value - 180.0;
}

inline double vector_angle_deg(const std::array<double, 2>& vec) {
    return std::atan2(vec[1], vec[0]) * 180.0 / kPi;
}

inline double norm2(const std::array<double, 2>& vec) {
    return std::sqrt(vec[0] * vec[0] + vec[1] * vec[1]);
}

/// Multiplies two `(w, x, y, z)` quaternions as `lhs * rhs`.
inline std::array<double, 4> quaternion_multiply(
    const std::array<double, 4>& lhs,
    const std::array<double, 4>& rhs) {
    return {
        lhs[0] * rhs[0] - lhs[1] * rhs[1] - lhs[2] * rhs[2] - lhs[3] * rhs[3],
        lhs[0] * rhs[1] + lhs[1] * rhs[0] + lhs[2] * rhs[3] - lhs[3] * rhs[2],
        lhs[0] * rhs[2] - lhs[1] * rhs[3] + lhs[2] * rhs[0] + lhs[3] * rhs[1],
        lhs[0] * rhs[3] + lhs[1] * rhs[2] - lhs[2] * rhs[1] + lhs[3] * rhs[0],
    };
}

inline std::array<double, 4> quaternion_conjugate(const std::array<double, 4>& q) {
    return {q[0], -q[1], -q[2], -q[3]};
}

/// Rotates a 3D vector by a unit `(w, x, y, z)` quaternion.
inline std::array<double, 3> rotate_vec_by_quaternion(
    const std::array<double, 3>& vec,
    const std::array<double, 4>& q) {
    const std::array<double, 4> vec_q{0.0, vec[0], vec[1], vec[2]};
    const auto rotated = quaternion_multiply(quaternion_multiply(q, vec_q), quaternion_conjugate(q));
    return {rotated[1], rotated[2], rotated[3]};
}

inline std::array<double, 3> rotate_z(const std::array<double, 3>& v, double yaw_deg) {
    const double yaw_rad = deg_to_rad(yaw_deg);
    const double c = std::cos(yaw_rad);
    const double s = std::sin(yaw_rad);
    return {c * v[0] - s * v[1], s * v[0] + c * v[1], v[2]};
}

inline std::array<double, 3> rotate_y(const std::array<double, 3>& v, double pitch_deg) {
    const double pitch_rad = deg_to_rad(pitch_deg);
    const double c = std::cos(pitch_rad);
    const double s = std::sin(pitch_rad);
    return {c * v[0] + s * v[2], v[1], -s * v[0] + c * v[2]};
}

inline std::array<double, 3> vec3_add(const std::array<double, 3>& a, const std::array<double, 3>& b) {
    return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}

inline std::array<double, 3> vec3_sub(const std::array<double, 3>& a, const std::array<double, 3>& b) {
    return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}

inline std::array<double, 3> vec3_scale(const std::array<double, 3>& v, double scalar) {
    return {v[0] * scalar, v[1] * scalar, v[2] * scalar};
}

inline std::array<double, 3> vec3_divide(const std::array<double, 3>& v, double scalar) {
    return {v[0] / scalar, v[1] / scalar, v[2] / scalar};
}

inline double norm3(const std::array<double, 3>& v) {
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

/// Composes yaw about +z followed by pitch about +y.
inline std::array<double, 4> yaw_pitch_quaternion(double yaw_deg, double pitch_deg) {
    const double yaw_rad = deg_to_rad(yaw_deg) * 0.5;
    const double pitch_rad = deg_to_rad(pitch_deg) * 0.5;
    const std::array<double, 4> yaw_q{std::cos(yaw_rad), 0.0, 0.0, std::sin(yaw_rad)};
    const std::array<double, 4> pitch_q{std::cos(pitch_rad), 0.0, std::sin(pitch_rad), 0.0};
    return quaternion_multiply(yaw_q, pitch_q);
}

inline double sq_dist2(const std::array<double, 2>& a, const std::array<double, 2>& b) {
    const double dx = a[0] - b[0];
    const double dy = a[1] - b[1];
    return dx * dx + dy * dy;
}

inline double planar_dist(const std::array<double, 2>& a, const std::array<double, 2>& b) {
    return std::sqrt(sq_dist2(a, b));
}

inline std::array<double, 2> rotate_2d(const std::array<double, 2>& vec, double angle_deg) {
    const double angle_rad = deg_to_rad(angle_deg);
    const double c = std::cos(angle_rad);
    const double s = std::sin(angle_rad);
    return {c * vec[0] - s * vec[1], s * vec[0] + c * vec[1]};
}

inline std::array<double, 2> vec2_add(const std::array<double, 2>& a, const std::array<double, 2>& b) {
    return {a[0] + b[0], a[1] + b[1]};
}

inline std::array<double, 2> vec2_sub(const std::array<double, 2>& a, const std::array<double, 2>& b) {
    return {a[0] - b[0], a[1] - b[1]};
}

inline std::array<double, 2> vec2_scale(const std::array<double, 2>& v, double scalar) {
    return {v[0] * scalar, v[1] * scalar};
}

inline std::array<double, 2> vec2_lerp(
    const std::array<double, 2>& a, const std::array<double, 2>& b, double t) {
    return {a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])};
}

/// Returns a unit vector, or `fallback` when `v` is nearly zero.
inline std::array<double, 2> vec2_unit_or(
    const std::array<double, 2>& v, const std::array<double, 2>& fallback) {
    const double n = norm2(v);
    if (n < 1e-6) {
        return fallback;
    }
    return {v[0] / n, v[1] / n};
}

inline std::array<double, 2> perpendicular_left(const std::array<double, 2>& v) {
    return {-v[1], v[0]};
}

inline std::array<double, 2> perpendicular_right(const std::array<double, 2>& v) {
    return {v[1], -v[0]};
}

/// Returns the Euclidean distance from a point to the closed segment `[a, b]`.
inline double point_segment_distance(
    const std::array<double, 2>& point,
    const std::array<double, 2>& a,
    const std::array<double, 2>& b) {
    const double vx = b[0] - a[0];
    const double vy = b[1] - a[1];
    const double len_sq = vx * vx + vy * vy;
    if (len_sq < 1e-12) {
        return planar_dist(point, a);
    }
    const double t = std::clamp(
        ((point[0] - a[0]) * vx + (point[1] - a[1]) * vy) / len_sq, 0.0, 1.0);
    return planar_dist(point, {a[0] + t * vx, a[1] + t * vy});
}

}  // namespace math
