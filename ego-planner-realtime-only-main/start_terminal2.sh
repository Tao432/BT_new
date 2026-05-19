#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common_env.sh"

prepare_ros_shell
require_file "${TOPIC_RELAY_SCRIPT}" "topic_relay.py 不存在"
ensure_system_python

log "启动 topic_relay.py"
exec "${SYSTEM_PYTHON}" "${TOPIC_RELAY_SCRIPT}" "$@"
