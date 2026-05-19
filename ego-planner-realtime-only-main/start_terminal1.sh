#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common_env.sh"

prepare_ros_shell
ensure_mujoco_python
require_file "${SCENE_SCRIPT}" "MuJoCo 场景脚本不存在"
require_file "${GO2_RVIZ_CONFIG}" "RViz 配置不存在"

if [[ $# -eq 0 ]]; then
  log "启动 MuJoCo + ROBOCON 场景，默认 LiDAR: airy"
  exec "${MUJOCO_PYTHON}" "${SCENE_SCRIPT}" --lidar airy
else
  log "启动 MuJoCo + ROBOCON 场景: $*"
  exec "${MUJOCO_PYTHON}" "${SCENE_SCRIPT}" "$@"
fi
