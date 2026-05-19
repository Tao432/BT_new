#!/usr/bin/env bash

# 复制为 env.local.sh 后按实际路径修改，再执行：
#   source ./env.local.sh

export WORKSPACE_ROOT=/path/to/workspace
export SCENE_ROOT="${WORKSPACE_ROOT}/ROBOCON2026_Scene-main"
export EGO_WS_ROOT="${WORKSPACE_ROOT}/ego_planner_ws/ego-planner-swarm-ros2_version"

# 推荐做法：
# 1. 在仓库根目录执行 ./setup_local_mujoco_env.sh
# 2. 使用仓库内 .venv/bin/python 作为 MuJoCo Python
# 如果你的 MuJoCo Python 环境不在默认位置，取消注释并修改：
# export MUJOCO_PYTHON=/path/to/repo/.venv/bin/python

# 如果 ROS setup.bash 不在默认位置，取消注释并修改：
# export ROS_SETUP=/opt/ros/jazzy/setup.bash
