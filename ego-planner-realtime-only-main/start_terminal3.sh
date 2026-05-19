#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common_env.sh"

prepare_ego_shell
require_file "${TERRAIN_FILTER_SCRIPT}" "terrain_filter.py 不存在"
require_file "${LOCAL_LAUNCH_FILE}" "本地 launch 文件不存在"

START_TOPIC_RELAY="${START_TOPIC_RELAY:-false}"
START_TERRAIN_FILTER="${START_TERRAIN_FILTER:-true}"
CLOUD_TOPIC_NAME="${CLOUD_TOPIC_NAME:-cloud_filtered}"

log "启动仅实时雷达版 EGO-Planner，外部 topic_relay + terrain_filter 模式"
exec ros2 launch "${LOCAL_LAUNCH_FILE}" \
  start_topic_relay:="${START_TOPIC_RELAY}" \
  start_terrain_filter:="${START_TERRAIN_FILTER}" \
  cloud_topic_name:="${CLOUD_TOPIC_NAME}" \
  "$@"
