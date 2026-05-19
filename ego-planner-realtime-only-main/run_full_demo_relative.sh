#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "${SCRIPT_DIR}"

export WORKSPACE_ROOT="${SCRIPT_DIR}"
export SCENE_ROOT="${SCRIPT_DIR}/ROBOCON2026_Scene-main"
export EGO_WS_ROOT="${SCRIPT_DIR}/ego_planner_ws/ego-planner-swarm-ros2_version"
export ROS_LOG_DIR="${SCRIPT_DIR}/logs/ros"

mkdir -p "${SCRIPT_DIR}/logs"
mkdir -p "${ROS_LOG_DIR}"

GOAL_X="${1:-2.0}"
GOAL_Y="${2:-0.5}"
GOAL_Z="${3:-0.3}"

echo "[run_full_demo_relative] repo root: ${SCRIPT_DIR}"
echo "[run_full_demo_relative] using bundled scene: ${SCENE_ROOT}"
echo "[run_full_demo_relative] using bundled ego ws: ${EGO_WS_ROOT}"

if [[ ! -f "${EGO_WS_ROOT}/install/setup.bash" ]]; then
  echo "[run_full_demo_relative] install/setup.bash missing, building bundled EGO workspace..."
  ./build_ego_planner.sh
fi

cleanup() {
  local exit_code=$?
  if [[ -n "${PID_T1:-}" ]]; then kill "${PID_T1}" 2>/dev/null || true; fi
  if [[ -n "${PID_T2:-}" ]]; then kill "${PID_T2}" 2>/dev/null || true; fi
  if [[ -n "${PID_T3:-}" ]]; then kill "${PID_T3}" 2>/dev/null || true; fi
  if [[ -n "${PID_T4:-}" ]]; then kill "${PID_T4}" 2>/dev/null || true; fi
  wait 2>/dev/null || true
  exit "${exit_code}"
}
trap cleanup INT TERM EXIT

./start_terminal1.sh >"${SCRIPT_DIR}/logs/terminal1.log" 2>&1 &
PID_T1=$!
sleep 3

./start_terminal2.sh >"${SCRIPT_DIR}/logs/terminal2.log" 2>&1 &
PID_T2=$!
sleep 2

./start_terminal3.sh >"${SCRIPT_DIR}/logs/terminal3.log" 2>&1 &
PID_T3=$!
sleep 5

./start_terminal4.sh >"${SCRIPT_DIR}/logs/terminal4.log" 2>&1 &
PID_T4=$!
sleep 2

./send_goal_once.sh "${GOAL_X}" "${GOAL_Y}" "${GOAL_Z}" >"${SCRIPT_DIR}/logs/send_goal.log" 2>&1 || true

echo "[run_full_demo_relative] started."
echo "[run_full_demo_relative] goal: x=${GOAL_X} y=${GOAL_Y} z=${GOAL_Z}"
echo "[run_full_demo_relative] logs:"
echo "  logs/terminal1.log"
echo "  logs/terminal2.log"
echo "  logs/terminal3.log"
echo "  logs/terminal4.log"
echo "  logs/send_goal.log"
echo "[run_full_demo_relative] press Ctrl+C to stop all processes."

wait
