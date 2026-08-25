// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 The Hong Kong University of Science and Technology (Guangzhou), Humanoid Computing & Learning Lab.

#pragma once

#include <array>
#include <optional>
#include <string>
#include <vector>

/// Raw perception protocol types; world-frame normalization happens downstream.
namespace server {

/// Spherical observation in sensor-relative meters and degrees.
struct PolarObservation {
    double distance_m{0.0};
    double azimuth_deg{0.0};
    double elevation_deg{0.0};
};

/// Game-controller state reported by the server.
struct GameStatePerception {
    double play_time{0.0};
    std::string play_mode;
    std::optional<std::string> team_left;
    std::optional<std::string> team_right;
    int score_left{0};
    int score_right{0};
};

struct JointStatePerception {
    std::string name;
    double ax_deg{0.0};
    double vx_deg_s{0.0};
};

struct GyroPerception {
    std::string name;
    std::array<double, 3> rot_deg_s{0.0, 0.0, 0.0};
};

struct AccelerometerPerception {
    std::string name;
    std::array<double, 3> accel_mps2{0.0, 0.0, 0.0};
};

struct PositionPerception {
    std::string name;
    std::array<double, 3> xyz_m{0.0, 0.0, 0.0};
};

struct QuaternionPerception {
    std::string name;
    std::array<double, 4> wxyz{1.0, 0.0, 0.0, 0.0};
};

/// One speech payload received from another player.
struct MicrophoneMessage {
    int azimuth_deg{0};
    std::string message_b64;
};

struct VisionMarkerObservation {
    std::string marker_name;
    PolarObservation polar;
};

struct BallVisionDetection {
    PolarObservation polar;
};

struct LandmarkVisionDetection {
    std::string object_type;
    PolarObservation polar;
};

struct PlayerVisionDetection {
    std::string team_name;
    int player_number{0};
    std::vector<VisionMarkerObservation> markers;
};

struct VisionPerception {
    std::optional<BallVisionDetection> ball;
    std::vector<LandmarkVisionDetection> landmarks;
    std::vector<PlayerVisionDetection> players;
};

/// All typed values decoded from one server perception frame.
struct PerceptionFrame {
    double server_time{0.0};
    int server_cycle{0};
    std::optional<GameStatePerception> game_state;
    std::vector<JointStatePerception> joints;
    std::optional<GyroPerception> gyro;
    std::optional<AccelerometerPerception> accel;
    std::optional<PositionPerception> position;
    std::optional<QuaternionPerception> orientation;
    std::vector<MicrophoneMessage> microphone_messages;
    VisionPerception vision;
};

}  // namespace server
