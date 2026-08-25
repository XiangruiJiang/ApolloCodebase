#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (c) 2026 The Hong Kong University of Science and Technology (Guangzhou), Humanoid Computing & Learning Lab.

set -euo pipefail

pid_file="/tmp/apollo_code_base_mj_team.pids"

if [[ -f "$pid_file" ]]; then
  while IFS= read -r pid; do
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      kill "$pid" || true
    fi
  done < "$pid_file"
  rm -f "$pid_file"
fi

pkill -f "ApolloCodeBase.*assets" 2>/dev/null || true
