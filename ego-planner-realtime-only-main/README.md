# ROBOCON EGO Realtime Only

这个仓库把之前测试过的短路径规划链路整理成了一个可单独分发的项目，包含三部分：

1. ROBOCON MuJoCo 场景
2. EGO-Planner 源码工作区
3. 实时雷达桥接、地形过滤和下位机控制桥接

目标是让这条链路既能在 MuJoCo 里复现，也能在实体机器人上沿用同一套规划 / 桥接接口：

```text
3D lidar + TF
  -> topic_relay.py
  -> terrain_filter.py
  -> ego_planner
  -> poscmd_2_odom
  -> pos_to_vel_bridge.py
  -> geometry_msgs/Twist
```

这意味着：

- 在仿真里，`start_terminal1.sh` 提供传感器和 TF
- 在实体机上，只需要把“终端 1 的仿真输入”替换成真实 3D 雷达和真实 TF
- 从 `topic_relay.py` 往后的路径规划和控制桥接逻辑保持一致

## 仓库内容

```text
robocon_ego_realtime_only/
  README.md
  ENVIRONMENT.md
  env.example.sh
  requirements-mujoco.txt
  setup_local_mujoco_env.sh
  build_ego_planner.sh
  check_environment.sh
  run_full_demo_relative.sh
  common_env.sh
  robocon_integration_realtime.launch.py
  topic_relay.py
  terrain_filter.py
  pos_to_vel_bridge.py
  start_terminal1.sh
  start_terminal2.sh
  start_terminal3.sh
  start_terminal4.sh
  start_rviz_clean.sh
  send_goal_once.sh
  set_terrain_mode.sh
  ROBOCON2026_Scene-main/
  ego_planner_ws/
    ego-planner-swarm-ros2_version/
      src/
```

仓库内已经包含：

- 场景代码和模型：`./ROBOCON2026_Scene-main`
- 路径规划源码：`./ego_planner_ws/ego-planner-swarm-ros2_version`

没有提交这些本地产物：

- `build/`
- `install/`
- `log/`
- `__pycache__/`
- `.venv/`

## 环境配置

### 1. 系统依赖

当前验证环境：

- Ubuntu 24.04
- ROS 2 Jazzy
- Python 3.12

需要能正常执行：

```bash
source /opt/ros/jazzy/setup.bash
```

### 2. 配置本地 MuJoCo Python 环境

这版 README 不再依赖 `/mnt/newspace` 挂载目录。  
推荐直接在仓库根目录创建本地 `.venv`：

```bash
cd <repo-root>
./setup_local_mujoco_env.sh
```

这个脚本会：

- 创建 `./.venv`
- 以 `--system-site-packages` 方式继承系统 ROS Python 包
- 安装 [requirements-mujoco.txt](./requirements-mujoco.txt) 里的 MuJoCo 运行依赖

当前 Python 依赖列表：

- `mujoco==3.4.0`
- `numpy==1.26.4`
- `scipy==1.17.0`
- `onnxruntime==1.23.2`
- `pygame==2.6.1`
- `etils==1.13.0`

如果你已经有自己的 Python 环境，也可以直接指定：

```bash
export MUJOCO_PYTHON=/your/python/path
```

### 3. 编译内置 EGO 工作区

仓库内包含的是源码工作区，所以第一次运行前需要编译：

```bash
cd <repo-root>
./build_ego_planner.sh
```

编译成功后会生成：

```text
./ego_planner_ws/ego-planner-swarm-ros2_version/install/setup.bash
```

### 4. 环境检查

```bash
cd <repo-root>
./check_environment.sh
```

这个检查会确认：

- ROS 环境可用
- MuJoCo Python 环境可用
- 场景脚本和模型文件存在
- 内置 EGO install 空间存在
- launch 参数可解析

### 5. 可选的环境变量覆盖

默认情况下，[common_env.sh](./common_env.sh) 会优先使用仓库内置路径：

- `./ROBOCON2026_Scene-main`
- `./ego_planner_ws/ego-planner-swarm-ros2_version`
- `./.venv/bin/python`

如果你想改路径，可以：

```bash
export WORKSPACE_ROOT=/your/workspace
export SCENE_ROOT=/your/workspace/ROBOCON2026_Scene-main
export EGO_WS_ROOT=/your/workspace/ego_planner_ws/ego-planner-swarm-ros2_version
export MUJOCO_PYTHON=/your/python/path
export ROS_SETUP=/opt/ros/jazzy/setup.bash
```

或者复制示例文件：

```bash
cp env.example.sh env.local.sh
source ./env.local.sh
```

## 如何运行示例

### 方式 1：一条命令试运行完整项目

```bash
cd <repo-root>
./run_full_demo_relative.sh 2.0 0.5 0.3
```

这个脚本会：

- 强制使用仓库内相对路径
- 检查并在需要时编译内置 EGO 工作区
- 启动 1 到 4 号流程
- 发送一次目标点

日志会写到：

```text
./logs/terminal1.log
./logs/terminal2.log
./logs/terminal3.log
./logs/terminal4.log
./logs/send_goal.log
```

### 方式 2：手动四终端运行

终端 1：

```bash
cd <repo-root>
./start_terminal1.sh
```

终端 2：

```bash
cd <repo-root>
./start_terminal2.sh
```

终端 3：

```bash
cd <repo-root>
./start_terminal3.sh
```

终端 4：

```bash
cd <repo-root>
./start_terminal4.sh
```

发目标：

```bash
cd <repo-root>
./send_goal_once.sh 2.0 0.5 0.3
```

如果 RViz 没自动出来：

```bash
cd <repo-root>
./start_rviz_clean.sh
```

## 3D 雷达接口如何对接

### 1. 实体机需要提供什么

最小输入是两类：

- 3D 雷达点云：`sensor_msgs/msg/PointCloud2`
- 一条完整 TF 链，让系统能从规划世界系推到机器人机体和雷达

推荐最小 TF 树：

```text
world -> odom   (可由 topic_relay 自动发布 identity)
odom  -> base_link 或 imu
base_link/imu -> lidar
```

常见的两种链都可以：

```text
odom -> base_link -> lidar
```

或：

```text
odom -> imu -> lidar
```

### 2. `topic_relay.py` 暴露的输入参数

[topic_relay.py](./topic_relay.py) 支持这些参数：

- `--input-cloud-topic`
- `--output-odom-topic`
- `--output-cloud-topic`
- `--odom-parent-frame`
- `--odom-child-frame`
- `--world-frame`
- `--publish-world-to-odom-tf`
- `--cloud-frame-override`

作用是把真实 3D 雷达和 TF 统一整理成规划器使用的标准输入：

- `/drone_0_odom`
- `/drone_0_cloud`

### 3. 实体机雷达接入示例

假设你的机器人提供：

- 点云：`/livox/lidar`
- TF：`odom -> base_link`
- TF：`base_link -> lidar`

那么 2 号终端可以这样启动：

```bash
cd <repo-root>
./start_terminal2.sh \
  --input-cloud-topic /livox/lidar \
  --output-odom-topic /drone_0_odom \
  --output-cloud-topic /drone_0_cloud \
  --odom-parent-frame odom \
  --odom-child-frame base_link \
  --world-frame world \
  --publish-world-to-odom-tf true
```

如果点云消息里没有正确的 `frame_id`，可以补一个：

```bash
./start_terminal2.sh \
  --input-cloud-topic /livox/lidar \
  --cloud-frame-override lidar \
  --odom-parent-frame odom \
  --odom-child-frame base_link \
  --world-frame world
```

### 4. `topic_relay.py` 输出约定

输出接口固定为：

- `/drone_0_odom`
  - 类型：`nav_msgs/msg/Odometry`
- `/drone_0_cloud`
  - 类型：`sensor_msgs/msg/PointCloud2`

从这里开始，仿真和实体机走同一条链路：

```text
/drone_0_odom + /drone_0_cloud
  -> terrain_filter.py
  -> ego_planner
```

## 速度接口如何对接

### 1. `pos_to_vel_bridge.py` 的作用

[pos_to_vel_bridge.py](./pos_to_vel_bridge.py) 负责把规划器输出的位置目标，转换成下位机可直接执行的速度命令。

它的输入是：

- 当前真实位置：默认 `/drone_0_odom`
- 规划目标 odom：默认 `/odometry`

它的输出是：

- `geometry_msgs/msg/Twist`

当前实际使用的字段是：

- `linear.x`
- `angular.z`

### 2. `pos_to_vel_bridge.py` 暴露的参数

- `--current-odom-topic`
- `--target-odom-topic`
- `--terrain-mode-topic`
- `--terrain-mode-active-topic`
- `--cmd-topic`

### 3. 下位机对接示例

如果你的下位机直接订阅标准 `cmd_vel`：

```bash
cd <repo-root>
./start_terminal4.sh \
  --current-odom-topic /drone_0_odom \
  --target-odom-topic /odometry \
  --cmd-topic /cmd_vel
```

如果你的下位机话题是 `/robot/cmd_vel`：

```bash
cd <repo-root>
./start_terminal4.sh \
  --current-odom-topic /drone_0_odom \
  --target-odom-topic /odometry \
  --cmd-topic /robot/cmd_vel
```

如果你的下位机不是 `Twist`，而是厂商自定义消息，那么建议保留当前 [pos_to_vel_bridge.py](./pos_to_vel_bridge.py) 不动，在它后面再加一个很薄的适配器：

```text
geometry_msgs/Twist -> your_robot_msgs/ControlCmd
```

这样可以保持仿真和实体机共用同一套短路径规划逻辑。

## 为什么这套短路径规划在实体机器人上仍然可行

这里不能保证“任何机器人拿来就能直接跑”，但这套仓库已经尽量把**和实体机相关的变化限制在接口层**，核心原因是：

1. 规划主链路没有改成 MuJoCo 专属逻辑  
   从 `/drone_0_odom` 和 `/drone_0_cloud` 开始，到 `ego_planner`、`poscmd_2_odom`、`pos_to_vel_bridge.py` 结束，这一段仿真和实体机共用。

2. 实体机只需要替换传感器入口  
   也就是把终端 1 提供的“仿真雷达 + TF”换成真实 3D 雷达和真实 TF。

3. 下位机输出保持标准接口  
   当前桥接输出是标准 `geometry_msgs/msg/Twist`，这让实际接轮式底盘、足式底盘或厂商控制器都更容易。

### 实体机使用时的必要前提

为了让短路径规划在实体机上稳定工作，仍然需要满足这些条件：

- `odom -> body -> lidar` 的 TF 稳定且方向正确
- 雷达点云坐标系正确
- `/drone_0_odom` 是实时真实位置，不是假的轨迹 odom
- 下位机能稳定跟随 `linear.x` / `angular.z`
- 机器人本体的最大速度、转弯能力、坡面能力和当前参数匹配

### 实体机最小运行顺序

实体机上通常不需要 `start_terminal1.sh`。

只要外部已经启动了：

- 真实定位
- 真实 TF 树
- 真实 3D 雷达驱动

就可以按这个顺序：

终端 2：

```bash
cd <repo-root>
./start_terminal2.sh \
  --input-cloud-topic /livox/lidar \
  --odom-parent-frame odom \
  --odom-child-frame base_link \
  --world-frame world
```

终端 3：

```bash
cd <repo-root>
./start_terminal3.sh
```

终端 4：

```bash
cd <repo-root>
./start_terminal4.sh \
  --current-odom-topic /drone_0_odom \
  --target-odom-topic /odometry \
  --cmd-topic /robot/cmd_vel
```

发目标：

```bash
cd <repo-root>
./send_goal_once.sh 2.0 0.5 0.3
```

## 最小检查命令

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic echo --once /drone_0_odom
ros2 topic echo --once /drone_0_cloud
ros2 topic echo --once /drone_0_cloud_filtered
ros2 topic echo --once /odometry
ros2 topic echo --once /cmd_vel
```

## 上传到 GitHub 之后的推荐使用顺序

1. 克隆仓库
2. `./setup_local_mujoco_env.sh`
3. `./build_ego_planner.sh`
4. `./check_environment.sh`
5. `./run_full_demo_relative.sh 2.0 0.5 0.3`
6. 再切换到实体机接口模式
