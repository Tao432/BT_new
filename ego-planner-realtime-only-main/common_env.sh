#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="${SCRIPT_DIR}"
DEFAULT_EXTERNAL_WORKSPACE_ROOT="$(cd "${PROJECT_ROOT}/.." && pwd)"
WORKSPACE_ROOT="${WORKSPACE_ROOT:-${PROJECT_ROOT}}"

if [[ -z "${SCENE_ROOT:-}" ]]; then
  if [[ -d "${PROJECT_ROOT}/ROBOCON2026_Scene-main" ]]; then
    SCENE_ROOT="${PROJECT_ROOT}/ROBOCON2026_Scene-main"
  else
    SCENE_ROOT="${DEFAULT_EXTERNAL_WORKSPACE_ROOT}/ROBOCON2026_Scene-main"
  fi
fi

if [[ -z "${EGO_WS_ROOT:-}" ]]; then
  if [[ -d "${PROJECT_ROOT}/ego_planner_ws/ego-planner-swarm-ros2_version" ]]; then
    EGO_WS_ROOT="${PROJECT_ROOT}/ego_planner_ws/ego-planner-swarm-ros2_version"
  else
    EGO_WS_ROOT="${DEFAULT_EXTERNAL_WORKSPACE_ROOT}/ego_planner_ws/ego-planner-swarm-ros2_version"
  fi
fi

SCENE_SCRIPT="${SCENE_SCRIPT:-${SCENE_ROOT}/src/robots/play_go2_ros2.py}"
TOPIC_RELAY_SCRIPT="${TOPIC_RELAY_SCRIPT:-${PROJECT_ROOT}/topic_relay.py}"
TERRAIN_FILTER_SCRIPT="${TERRAIN_FILTER_SCRIPT:-${PROJECT_ROOT}/terrain_filter.py}"
POS_TO_VEL_BRIDGE_SCRIPT="${POS_TO_VEL_BRIDGE_SCRIPT:-${PROJECT_ROOT}/pos_to_vel_bridge.py}"
LOCAL_LAUNCH_FILE="${LOCAL_LAUNCH_FILE:-${PROJECT_ROOT}/robocon_integration_realtime.launch.py}"
GO2_RVIZ_CONFIG="${GO2_RVIZ_CONFIG:-${SCENE_ROOT}/src/rviz_config/go2.rviz}"
EGO_INSTALL_SETUP="${EGO_INSTALL_SETUP:-${EGO_WS_ROOT}/install/setup.bash}"
SYSTEM_PYTHON="${SYSTEM_PYTHON:-/usr/bin/python3}"
ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/roslog}"
ROS_PREFIX="${ROS_PREFIX:-}"

if [[ -z "${MUJOCO_PYTHON:-}" ]]; then
  if [[ -x "${PROJECT_ROOT}/.venv/bin/python" ]]; then
    MUJOCO_PYTHON="${PROJECT_ROOT}/.venv/bin/python"
  elif [[ -x "${PROJECT_ROOT}/mujoco_env/bin/python" ]]; then
    MUJOCO_PYTHON="${PROJECT_ROOT}/mujoco_env/bin/python"
  elif [[ -x "/mnt/newspace/mujoco_env/bin/python" ]]; then
    MUJOCO_PYTHON="/mnt/newspace/mujoco_env/bin/python"
  else
    MUJOCO_PYTHON="${PROJECT_ROOT}/.venv/bin/python"
  fi
fi

log() {
  printf '[robocon_ego_bridge] %s\n' "$*"
}

fail() {
  printf '[robocon_ego_bridge] %s\n' "$*" >&2
  exit 1
}

require_file() {
  local file_path="$1"
  local hint="$2"
  [[ -e "${file_path}" ]] || fail "${hint}: ${file_path}"
}

source_compat() {
  local file_path="$1"
  local nounset_was_on=0

  if [[ $- == *u* ]]; then
    nounset_was_on=1
    set +u
  fi

  # shellcheck disable=SC1090
  source "${file_path}"

  if [[ "${nounset_was_on}" -eq 1 ]]; then
    set -u
  fi
}

resolve_ros_setup() {
  if [[ -n "${ROS_SETUP:-}" ]]; then
    printf '%s\n' "${ROS_SETUP}"
    return
  fi

  if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    printf '%s\n' "/opt/ros/${ROS_DISTRO}/setup.bash"
    return
  fi

  if [[ -f /opt/ros/rolling/setup.bash ]]; then
    printf '%s\n' /opt/ros/rolling/setup.bash
    return
  fi

  if [[ -f /opt/ros/jazzy/setup.bash ]]; then
    printf '%s\n' /opt/ros/jazzy/setup.bash
    return
  fi

  fail "找不到可用的 ROS setup.bash。请设置 ROS_SETUP，或确认 /opt/ros/jazzy/setup.bash 或 /opt/ros/rolling/setup.bash 存在。"
}

deactivate_conda_if_needed() {
  local conda_sh="${HOME}/miniconda3/etc/profile.d/conda.sh"
  if [[ -f "${conda_sh}" ]]; then
    source_compat "${conda_sh}"
    while [[ -n "${CONDA_SHLVL:-}" && "${CONDA_SHLVL}" -gt 0 ]]; do
      conda deactivate || break
    done
  fi
}

prepare_ros_shell() {
  deactivate_conda_if_needed
  ROS_SETUP="$(resolve_ros_setup)"
  require_file "${ROS_SETUP}" "ROS setup.bash 不存在"
  if [[ -z "${ROS_PREFIX}" ]]; then
    ROS_PREFIX="$(cd "$(dirname "${ROS_SETUP}")/.." && pwd)"
  fi
  export ROS_PREFIX
  mkdir -p "${ROS_LOG_DIR}"
  export ROS_LOG_DIR
  source_compat "${ROS_SETUP}"
}

prepare_ego_shell() {
  prepare_ros_shell
  require_file "${EGO_INSTALL_SETUP}" "EGO-Planner install/setup.bash 不存在"
  source_compat "${EGO_INSTALL_SETUP}"
}

ensure_mujoco_python() {
  if [[ ! -x "${MUJOCO_PYTHON}" ]]; then
    fail "找不到 MuJoCo Python: ${MUJOCO_PYTHON}
推荐先在仓库根目录执行:
  ./setup_local_mujoco_env.sh
如果你已经有自己的 MuJoCo Python 环境，也可以设置:
  export MUJOCO_PYTHON=/your/python/path"
  fi
}

ensure_system_python() {
  if [[ ! -x "${SYSTEM_PYTHON}" ]]; then
    fail "找不到系统 Python: ${SYSTEM_PYTHON}"
  fi
}
