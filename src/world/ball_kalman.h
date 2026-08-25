// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 The Hong Kong University of Science and Technology (Guangzhou), Humanoid Computing & Learning Lab.

#pragma once

#include "src/server/server_constants.h"     // kBallRadiusM
#include "src/world/frame_normalizer.h"  // Vec3

#include <algorithm>
#include <cmath>

namespace world {

/// Constant-velocity ball filter in the canonical frame (own goal at `-x`).
///
/// The x and y axes are filtered independently. Measurement noise scales with
/// observation distance, while a large innovation reinitializes the track.
/// Parameters follow the RCSSServerMJ perception benchmark.
class BallKalman {
public:
    /// One decoupled axis with state `[position, velocity]` and 2x2 covariance.
    struct Axis {
        double p{0.0};
        double v{0.0};
        double p00{0.25}, p01{0.0}, p10{0.0}, p11{4.0};
        bool initialized{false};

        void reset(double z) {
            p = z;
            v = 0.0;
            p00 = 0.25; p01 = 0.0; p10 = 0.0; p11 = 4.0;
            initialized = true;
        }

        void predict(double dt, double accel_var) {
            // x = F x, with F = [[1, dt], [0, 1]].
            p += dt * v;
            // P = F P F^T + Q.
            const double n00 = p00 + dt * (p01 + p10) + dt * dt * p11;
            const double n01 = p01 + dt * p11;
            const double n10 = p10 + dt * p11;
            const double n11 = p11;
            const double dt2 = dt * dt;
            const double dt3 = dt2 * dt;
            p00 = n00 + accel_var * dt3 / 3.0;
            p01 = n01 + accel_var * dt2 / 2.0;
            p10 = n10 + accel_var * dt2 / 2.0;
            p11 = n11 + accel_var * dt;
        }

        void update(double z, double r) {
            // H = [1, 0], S = P00 + r, K = [P00/S, P10/S].
            const double s = p00 + r;
            const double k0 = p00 / s;
            const double k1 = p10 / s;
            const double y = z - p;
            p += k0 * y;
            v += k1 * y;
            const double n00 = (1.0 - k0) * p00;
            const double n01 = (1.0 - k0) * p01;
            const double n10 = p10 - k1 * p00;
            const double n11 = p11 - k1 * p01;
            p00 = n00; p01 = n01; p10 = n10; p11 = n11;
        }
    };

    bool has_track() const { return x_.initialized; }

    /// Returns whether velocity is fresh and sufficiently converged for consumers.
    bool velocity_confident(double now) const {
        return updates_ >= 2 && x_.p00 < 0.15 && y_.p00 < 0.15 &&
               (now - last_update_time_) <= kVelocityFreshHorizonS;
    }

    Vec3 position() const { return {x_.p, y_.p, z_}; }
    Vec3 velocity() const { return {x_.v, y_.v, 0.0}; }

    void reset() {
        x_.initialized = false;
        y_.initialized = false;
        z_ = server_constants::kBallRadiusM;
        updates_ = 0;
        last_update_time_ = -1.0;
    }

    void predict(double dt) {
        if (!x_.initialized || dt <= 0.0) {
            return;
        }
        const double accel_var = kAccelSigmaMps2 * kAccelSigmaMps2;
        x_.predict(dt, accel_var);
        y_.predict(dt, accel_var);
    }

    /// Updates from a gate-accepted real detection.
    ///
    /// `distance_m` is the self-to-ball range used for noise scaling. Synthesized
    /// fallback positions must not be passed to the filter.
    void update(const Vec3& measurement, double distance_m, double now) {
        if (!x_.initialized) {
            x_.reset(measurement[0]);
            y_.reset(measurement[1]);
            z_ = measurement[2];
            updates_ = 1;
            last_update_time_ = now;
            return;
        }
        const double dx = measurement[0] - x_.p;
        const double dy = measurement[1] - y_.p;
        if (std::sqrt(dx * dx + dy * dy) > kResetGateM) {
            x_.reset(measurement[0]);
            y_.reset(measurement[1]);
            z_ = measurement[2];
            updates_ = 1;
            last_update_time_ = now;
            return;
        }
        double r = kMeasNoisePerMeter * std::max(distance_m, kMinMeasurementRangeM);
        r *= r;
        x_.update(measurement[0], r);
        y_.update(measurement[1], r);
        z_ = 0.7 * z_ + 0.3 * measurement[2];  // light low-pass on height
        ++updates_;
        last_update_time_ = now;
    }

private:
    static constexpr double kMeasNoisePerMeter = 0.012;   // R = (k*d)^2
    static constexpr double kAccelSigmaMps2 = 3.0;        // process (kick) noise
    static constexpr double kResetGateM = 3.0;            // re-init on larger jumps
    static constexpr double kVelocityFreshHorizonS = 0.3;
    // Floor for noise-scaling range so a detection reported at near-zero
    // distance (ball on top of the robot, or comm-fused) doesn't collapse the
    // measurement sigma to zero.
    static constexpr double kMinMeasurementRangeM = 0.5;

    Axis x_;
    Axis y_;
    double z_{server_constants::kBallRadiusM};
    int updates_{0};
    double last_update_time_{-1.0};
};

}  // namespace world
