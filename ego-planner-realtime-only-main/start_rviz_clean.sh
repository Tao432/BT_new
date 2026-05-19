#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common_env.sh"

prepare_ros_shell
require_file "${GO2_RVIZ_CONFIG}" "RViz 配置不存在"

for key in \
  GTK_PATH \
  GTK_EXE_PREFIX \
  GIO_MODULE_DIR \
  GTK_IM_MODULE_FILE \
  QT_PLUGIN_PATH \
  QML2_IMPORT_PATH
do
  unset "${key}" || true
done

for key in $(env | awk -F= '/^SNAP/{print $1}'); do
  unset "${key}" || true
done

export PATH="${ROS_PREFIX}/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
export LD_LIBRARY_PATH="${ROS_PREFIX}/opt/rviz_ogre_vendor/lib:${ROS_PREFIX}/lib/x86_64-linux-gnu:${ROS_PREFIX}/opt/gz_math_vendor/lib:${ROS_PREFIX}/opt/gz_utils_vendor/lib:${ROS_PREFIX}/opt/gz_cmake_vendor/lib:${ROS_PREFIX}/lib"
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"

log "启动净化环境下的 rviz2"
exec rviz2 -d "${GO2_RVIZ_CONFIG}"
