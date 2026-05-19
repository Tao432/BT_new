#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common_env.sh"

prepare_ros_shell
ensure_system_python
ensure_mujoco_python

HAS_EGO_INSTALL=0
if [[ -f "${EGO_INSTALL_SETUP}" ]]; then
  source_compat "${EGO_INSTALL_SETUP}"
  HAS_EGO_INSTALL=1
fi

require_file "${SCENE_SCRIPT}" "MuJoCo 场景脚本不存在"
require_file "${TOPIC_RELAY_SCRIPT}" "topic_relay.py 不存在"
require_file "${TERRAIN_FILTER_SCRIPT}" "terrain_filter.py 不存在"
require_file "${POS_TO_VEL_BRIDGE_SCRIPT}" "pos_to_vel_bridge.py 不存在"
require_file "${LOCAL_LAUNCH_FILE}" "本地 launch 文件不存在"
require_file "${GO2_RVIZ_CONFIG}" "RViz 配置不存在"

echo "== 基础路径 =="
echo "ROS_SETUP=${ROS_SETUP}"
echo "ROS_PREFIX=${ROS_PREFIX}"
echo "SCENE_ROOT=${SCENE_ROOT}"
echo "EGO_WS_ROOT=${EGO_WS_ROOT}"
echo "EGO_INSTALL_SETUP=${EGO_INSTALL_SETUP}"
echo "MUJOCO_PYTHON=${MUJOCO_PYTHON}"
echo

if [[ "${HAS_EGO_INSTALL}" -eq 1 ]]; then
  echo "== EGO 工作空间 =="
  echo "install/setup.bash: ok"
else
  echo "== EGO 工作空间 =="
  echo "install/setup.bash: MISSING"
  echo "请先运行: ./build_ego_planner.sh"
fi
echo

echo "== ROS 包 =="
pkg_list="$(ros2 pkg list || true)"
for pkg in ego_planner poscmd_2_odom traj_utils; do
  if printf '%s\n' "${pkg_list}" | grep -Fxq "${pkg}"; then
    echo "${pkg}"
  else
    echo "${pkg}: MISSING"
  fi
done
echo

echo "== 系统 Python / ROS 导入 =="
"${SYSTEM_PYTHON}" - <<'PY'
import importlib
mods = ["rclpy", "numpy", "sensor_msgs", "nav_msgs"]
for m in mods:
    try:
        importlib.import_module(m)
        print(f"{m}: ok")
    except Exception as e:
        print(f"{m}: FAIL {e!r}")
PY
echo

echo "== MuJoCo Python / 仿真依赖导入 =="
"${MUJOCO_PYTHON}" - <<'PY'
import importlib
mods = [
    "mujoco",
    "numpy",
    "scipy",
    "onnxruntime",
    "pygame",
    "etils",
    "rclpy",
    "tf2_ros",
    "cv_bridge",
    "geometry_msgs.msg",
    "sensor_msgs.msg",
]
for m in mods:
    try:
        mod = importlib.import_module(m)
        ver = getattr(mod, "__version__", None)
        if ver is None and m == "pygame":
            ver = mod.version.ver
        print(f"{m}: ok version={ver}")
    except Exception as e:
        print(f"{m}: FAIL {e!r}")
PY
echo

echo "== 场景脚本导入 =="
(
  cd "${SCENE_ROOT}/src/robots"
  "${MUJOCO_PYTHON}" - <<'PY'
mods = ["camera_utils", "play_go2_joystick", "mujoco_lidar", "play_go2_ros2"]
for m in mods:
    try:
        __import__(m)
        print(f"{m}: ok")
    except Exception as e:
        print(f"{m}: FAIL {e!r}")
PY
)
echo

echo "== MuJoCo 模型加载 =="
(
  cd "${SCENE_ROOT}/src/robots"
  "${MUJOCO_PYTHON}" - <<'PY'
from pathlib import Path
import mujoco

path = Path("..") / ".." / "models" / "mjcf" / "scene_go2.xml"
model = mujoco.MjModel.from_xml_path(str(path))
data = mujoco.MjData(model)
print(f"scene_go2.xml loaded: nq={model.nq}, nv={model.nv}, nu={model.nu}, ngeom={model.ngeom}")
print(f"bodies={model.nbody}, cameras={model.ncam}")
PY
)
echo

echo "== Launch 参数检查 =="
if [[ "${HAS_EGO_INSTALL}" -eq 1 ]]; then
  ROS_LOG_DIR=/tmp/roslog ros2 launch "${LOCAL_LAUNCH_FILE}" --show-args >/tmp/robocon_realtime_launch_args.txt
  tail -n 20 /tmp/robocon_realtime_launch_args.txt
else
  echo "跳过 launch 参数检查：EGO install/setup.bash 尚未生成"
fi
echo

echo "环境检查完成。"
