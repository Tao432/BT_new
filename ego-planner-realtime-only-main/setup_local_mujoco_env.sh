#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
VENV_DIR="${SCRIPT_DIR}/.venv"
PYTHON_BIN="${PYTHON_BIN:-python3}"

if [[ ! -x "$(command -v "${PYTHON_BIN}")" ]]; then
  echo "[setup_local_mujoco_env] 找不到 Python: ${PYTHON_BIN}" >&2
  exit 1
fi

echo "[setup_local_mujoco_env] creating venv at ${VENV_DIR}"
"${PYTHON_BIN}" -m venv --system-site-packages "${VENV_DIR}"

echo "[setup_local_mujoco_env] upgrading pip"
"${VENV_DIR}/bin/python" -m pip install --upgrade pip

echo "[setup_local_mujoco_env] installing Python packages"
"${VENV_DIR}/bin/pip" install -r "${SCRIPT_DIR}/requirements-mujoco.txt"

echo
echo "[setup_local_mujoco_env] done"
echo "use this interpreter by default:"
echo "  export MUJOCO_PYTHON=${VENV_DIR}/bin/python"
echo "or just keep the generated .venv under repo root."
