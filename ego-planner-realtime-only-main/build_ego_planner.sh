#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common_env.sh"

prepare_ros_shell

require_file "${EGO_WS_ROOT}/src/planner/plan_manage/CMakeLists.txt" \
  "未找到 bundled ego_planner 源码，请确认仓库里的 ego_planner_ws 已存在"

log "编译 EGO-Planner 工作空间: ${EGO_WS_ROOT}"
cd "${EGO_WS_ROOT}"
colcon build --symlink-install

log "编译完成，可继续运行:"
log "  ./start_terminal3.sh"
