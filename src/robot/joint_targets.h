// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 The Hong Kong University of Science and Technology (Guangzhou), Humanoid Computing & Learning Lab.

#pragma once

#include <string>
#include <vector>

namespace robot {

/// Position-control target for one readable robot joint.
struct JointTarget {
    std::string joint_name;
    double q_deg{0.0};
    double dq_deg{0.0};
    double kp{0.0};
    double kd{0.0};
    double tau{0.0};
};

/// Complete set of joint targets emitted for one control cycle.
using JointTargets = std::vector<JointTarget>;

}  // namespace robot
