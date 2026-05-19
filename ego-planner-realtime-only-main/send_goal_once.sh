#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common_env.sh"

prepare_ros_shell

X="${1:-2.0}"
Y="${2:-0.5}"
Z="${3:-0.3}"

log "发送一次目标: x=${X}, y=${Y}, z=${Z}"
exec ros2 topic pub --once /move_base_simple/goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'world'}, pose: {position: {x: ${X}, y: ${Y}, z: ${Z}}, orientation: {w: 1.0}}}"
