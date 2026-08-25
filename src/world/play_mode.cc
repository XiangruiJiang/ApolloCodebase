// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 The Hong Kong University of Science and Technology (Guangzhou), Humanoid Computing & Learning Lab.

#include "src/world/play_mode.h"

#include <stdexcept>
#include <string_view>
#include <unordered_map>

namespace world {

namespace {

struct PlayModePair {
    PlayMode left_team;
    PlayMode right_team;
};

PlayMode select_for_side(const PlayModePair& pair, bool is_left_team) {
    return is_left_team ? pair.left_team : pair.right_team;
}

const std::unordered_map<std::string_view, PlayModePair>& play_mode_table() {
    static const std::unordered_map<std::string_view, PlayModePair> kTable{
        {"BeforeKickOff",          {PlayMode::BeforeKickOff,      PlayMode::BeforeKickOff}},
        {"PlayOn",                 {PlayMode::PlayOn,             PlayMode::PlayOn}},
        {"GameOver",               {PlayMode::GameOver,           PlayMode::GameOver}},
        {"KickOff_Left",           {PlayMode::OurKickOff,         PlayMode::TheirKickOff}},
        {"KickOff_Right",          {PlayMode::TheirKickOff,       PlayMode::OurKickOff}},
        {"KickIn_Left",            {PlayMode::OurThrowIn,         PlayMode::TheirThrowIn}},
        {"KickIn_Right",           {PlayMode::TheirThrowIn,       PlayMode::OurThrowIn}},
        {"corner_kick_left",       {PlayMode::OurCornerKick,      PlayMode::TheirCornerKick}},
        {"corner_kick_right",      {PlayMode::TheirCornerKick,    PlayMode::OurCornerKick}},
        {"goal_kick_left",         {PlayMode::OurGoalKick,        PlayMode::TheirGoalKick}},
        {"goal_kick_right",        {PlayMode::TheirGoalKick,      PlayMode::OurGoalKick}},
        {"offside_left",           {PlayMode::OurOffside,         PlayMode::TheirOffside}},
        {"offside_right",          {PlayMode::TheirOffside,       PlayMode::OurOffside}},
        {"Goal_Left",              {PlayMode::OurGoal,            PlayMode::TheirGoal}},
        {"Goal_Right",             {PlayMode::TheirGoal,          PlayMode::OurGoal}},
        {"free_kick_left",         {PlayMode::OurFreeKick,        PlayMode::TheirFreeKick}},
        {"free_kick_right",        {PlayMode::TheirFreeKick,      PlayMode::OurFreeKick}},
        {"direct_free_kick_left",  {PlayMode::OurDirectFreeKick,  PlayMode::TheirDirectFreeKick}},
        {"direct_free_kick_right", {PlayMode::TheirDirectFreeKick,PlayMode::OurDirectFreeKick}},
        {"penalty_kick_left",      {PlayMode::OurPenaltyKick,     PlayMode::TheirPenaltyKick}},
        {"penalty_kick_right",     {PlayMode::TheirPenaltyKick,   PlayMode::OurPenaltyKick}},
        {"penalty_shoot_left",     {PlayMode::OurPenaltyShoot,    PlayMode::TheirPenaltyShoot}},
        {"penalty_shoot_right",    {PlayMode::TheirPenaltyShoot,  PlayMode::OurPenaltyShoot}},
    };
    return kTable;
}

}  // namespace

PlayMode play_mode_from_token(const std::string& play_mode_token, bool is_left_team) {
    const auto& table = play_mode_table();
    const auto it = table.find(play_mode_token);
    if (it == table.end()) {
        throw std::invalid_argument("Unsupported play mode token: " + play_mode_token);
    }
    return select_for_side(it->second, is_left_team);
}

PlayModeGroup play_mode_group(PlayMode play_mode, bool is_left_team) {
    switch (play_mode) {
    case PlayMode::PlayOn:
    case PlayMode::GameOver:
        return PlayModeGroup::Other;
    case PlayMode::OurCornerKick:
    case PlayMode::OurDirectFreeKick:
    case PlayMode::OurFreeKick:
    case PlayMode::OurGoalKick:
    case PlayMode::OurKickOff:
    case PlayMode::OurOffside:
    case PlayMode::OurPenaltyKick:
    case PlayMode::OurPenaltyShoot:
    case PlayMode::OurThrowIn:
        return PlayModeGroup::OurKick;
    case PlayMode::TheirCornerKick:
    case PlayMode::TheirDirectFreeKick:
    case PlayMode::TheirFreeKick:
    case PlayMode::TheirGoalKick:
    case PlayMode::TheirKickOff:
    case PlayMode::TheirOffside:
    case PlayMode::TheirPenaltyKick:
    case PlayMode::TheirPenaltyShoot:
    case PlayMode::TheirThrowIn:
        return PlayModeGroup::TheirKick;
    case PlayMode::TheirGoal:
        return PlayModeGroup::ActiveBeam;
    case PlayMode::OurGoal:
        return PlayModeGroup::PassiveBeam;
    case PlayMode::BeforeKickOff:
        return is_left_team ? PlayModeGroup::ActiveBeam : PlayModeGroup::PassiveBeam;
    case PlayMode::NotInitialized:
        return PlayModeGroup::NotInitialized;
    }

    throw std::invalid_argument("Unsupported play mode for grouping");
}

bool should_reset_beam(PlayMode play_mode) {
    return play_mode == PlayMode::BeforeKickOff || play_mode == PlayMode::OurGoal ||
           play_mode == PlayMode::TheirGoal;
}

}  // namespace world
