// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 The Hong Kong University of Science and Technology (Guangzhou), Humanoid Computing & Learning Lab.

#pragma once

#include <string>

namespace world {

/// Team-relative form of the server play-mode token.
enum class PlayMode {
    NotInitialized,
    BeforeKickOff,
    OurKickOff,
    TheirKickOff,
    PlayOn,
    OurThrowIn,
    TheirThrowIn,
    OurCornerKick,
    TheirCornerKick,
    OurGoalKick,
    TheirGoalKick,
    OurOffside,
    TheirOffside,
    GameOver,
    OurGoal,
    TheirGoal,
    OurFreeKick,
    TheirFreeKick,
    OurDirectFreeKick,
    TheirDirectFreeKick,
    OurPenaltyKick,
    TheirPenaltyKick,
    OurPenaltyShoot,
    TheirPenaltyShoot,
};

/// Coarse play-mode category used by the behavior tree.
enum class PlayModeGroup {
    NotInitialized,
    Other,
    OurKick,
    TheirKick,
    ActiveBeam,
    PassiveBeam,
};

/// Converts a server token into a team-relative mode.
PlayMode play_mode_from_token(const std::string& play_mode_token, bool is_left_team);
PlayModeGroup play_mode_group(PlayMode play_mode, bool is_left_team);
bool should_reset_beam(PlayMode play_mode);

}  // namespace world
