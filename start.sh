#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (c) 2026 The Hong Kong University of Science and Technology (Guangzhou), Humanoid Computing & Learning Lab.

set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
binary="$repo_root/build/ApolloCodeBase"
pid_file="/tmp/apollo_code_base_mj_team.pids"

host="${1:-127.0.0.1}"
port="${2:-60000}"
team_name="${TEAM_NAME:-Apollo3Drelease}"

rm -f "$pid_file"
touch "$pid_file"

for i in $(seq 1 7); do
  "$binary" --team "$team_name" --player-number "$i" --host "$host" --port "$port" --asset-root assets >"apollo_code_base_mj_${i}.log" 2>&1 &
  echo $! >> "$pid_file"
  sleep 0.1
done
