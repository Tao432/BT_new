#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common_env.sh"

prepare_ros_shell
require_file "${POS_TO_VEL_BRIDGE_SCRIPT}" "pos_to_vel_bridge.py 不存在"
ensure_system_python

log "启动位置 -> 速度桥接"
exec "${SYSTEM_PYTHON}" "${POS_TO_VEL_BRIDGE_SCRIPT}" "$@"
