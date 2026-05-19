#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common_env.sh"

prepare_ros_shell

MODE="${1:-auto}"

case "${MODE}" in
  flat|ramp|auto)
    ;;
  *)
    echo "用法: $0 [flat|ramp|auto]" >&2
    exit 1
    ;;
esac

log "切换 terrain_mode -> ${MODE}"
exec ros2 topic pub --once /terrain_mode std_msgs/msg/String "{data: '${MODE}'}"
