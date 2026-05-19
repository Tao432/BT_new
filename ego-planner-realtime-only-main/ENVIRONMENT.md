# Environment Checklist

这份清单对应当前完整仓库版本：

- 内置 `ROBOCON2026_Scene-main`
- 内置 `ego_planner_ws/ego-planner-swarm-ros2_version` 源码工作区
- 内置实时雷达桥接与控制脚本

## 需要的运行环境

### 1. ROS 2

- 已验证发行版：`jazzy`
- 需要能 `source /opt/ros/jazzy/setup.bash`
- 需要 `ros2` 和 `rviz2`
- 需要 ROS Python 包：
  - `rclpy`
  - `tf2_ros`
  - `cv_bridge`
  - `geometry_msgs`
  - `sensor_msgs`
  - `nav_msgs`

### 2. EGO-Planner 工作空间

- 仓库内已包含源码工作区：
  - `./ego_planner_ws/ego-planner-swarm-ros2_version`
- 首次使用前需要编译：
  - `./build_ego_planner.sh`
- 编译后需要存在：
  - `./ego_planner_ws/ego-planner-swarm-ros2_version/install/setup.bash`

### 3. MuJoCo Python 环境

- 当前验证解释器：
  - `/mnt/newspace/mujoco_env/bin/python`
- 当前验证版本：
  - Python `3.12.12`
  - `mujoco 3.4.0`
  - `numpy 1.26.4`
  - `scipy 1.17.0`
  - `onnxruntime 1.23.2`
  - `pygame 2.6.1`
  - `etils 1.13.0`
  - `rclpy`
  - `tf2_ros`
  - `cv_bridge`

### 4. 场景与模型文件

- MuJoCo 场景脚本：
  - `./ROBOCON2026_Scene-main/src/robots/play_go2_ros2.py`
- MuJoCo 场景模型：
  - `./ROBOCON2026_Scene-main/models/mjcf/scene_go2.xml`
- RViz 配置：
  - `./ROBOCON2026_Scene-main/src/rviz_config/go2.rviz`

## 推荐检查顺序

1. 先执行：

```bash
./build_ego_planner.sh
```

2. 再执行：

```bash
./check_environment.sh
```

## 已做的冒烟测试

这些检查在当前机器上已经通过：

1. `mujoco_env` 能导入：
   - `mujoco`
   - `onnxruntime`
   - `rclpy`
   - `tf2_ros`
   - `cv_bridge`
2. `play_go2_ros2.py` 可被直接导入
3. `scene_go2.xml` 能被 `mujoco.MjModel.from_xml_path()` 正常加载
4. 实时雷达版 launch 的 `--show-args` 正常
5. `topic_relay.py --help` 和 `pos_to_vel_bridge.py --help` 正常

## 当前结论

从源码完整性、环境依赖和模型加载角度看，这个仓库已经包含了之前测试功能所需的主要代码。

第一次在新机器上使用时，仍然需要：

1. 准备 ROS 2 Jazzy
2. 准备 MuJoCo Python 环境
3. 编译仓库内置的 EGO 工作区

## 说明

- `rg` 不是必需依赖
- `check_environment.sh` 现在只依赖常见的 `grep`
- 编译产物 `build/ install/ log/` 没有提交进仓库，需要本地生成
