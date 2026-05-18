# Alphabata Tactical AI & Vision System

本项目是一个基于 ROS 2 (Jazzy) 的战术井字棋机器人视觉感知与 AI 决策系统。通过外接 USB 摄像头识别棋盘状态，利用 YOLO 模型提取特征，并使用 Alpha-Beta 剪枝算法进行实时博弈决策。

---

## 🕹️ 常用快捷指令 (快速复制区)

以下指令均假设您已打开一个新的终端。

**1. 🚀 一键启动系统 (包含驱动、视觉、AI)**
\`\`\`bash
cd ~/Alphabata_ws && ./start.sh
\`\`\`

**2. 📺 查看实时监控画面 (YOLO检测框)**
\`\`\`bash
conda deactivate && source /opt/ros/jazzy/setup.bash
ros2 run rqt_image_view rqt_image_view
\`\`\`
*(注：打开后，请在左上角下拉菜单中选择 `/vision/result_image`)*

**3. 🧠 查看 AI 决策与棋盘矩阵 (终端输出)**
\`\`\`bash
conda deactivate && source /opt/ros/jazzy/setup.bash
ros2 topic echo /vision/matrix
\`\`\`

---

## 📂 核心模块体系

本工作空间下的 `AAAmodel` 包包含以下核心文件结构与节点：

* **视觉感知层 (`vision_node.py`)**
  * **节点名称**: `yolo_detector`
  * **功能**: 订阅摄像头图像，调用本地 `.pt` 权重文件进行推理，提取并发布 3x3 棋盘颜色矩阵状态 (`/vision/matrix`) 以及可视化渲染图像 (`/vision/result_image`)。
* **逻辑决策层 (`ai_node.py`)**
  * **节点名称**: `tactical_ai`
  * **功能**: 订阅视觉节点发布的矩阵数据，实例化 `game.py` 中的博弈环境，并调用 `AlphaBataBot.py` 进行深度树搜索，输出最佳落子策略。
* **启动管理 (`start_game.launch.py`)**
  * 统筹管理所有节点，支持一键拉起底层硬件驱动 (`v4l2_camera`)、视觉节点和 AI 决策节点。

## 🛠️ 环境依赖

* **操作系统**: Ubuntu 24.04
* **ROS 版本**: ROS 2 Jazzy
* **硬件设备**: 外部 USB 摄像头 (默认挂载于 `/dev/video2`)
* **Python 依赖**: `ultralytics`, `opencv-python`, `numpy` (确保在系统环境中安装，而非 Conda 环境)

## 🚀 编译指南

每次修改 Python 代码后，请在工作空间根目录重新编译以更新 `install` 文件夹：
\`\`\`bash
# 务必确保已退出 conda 环境
conda deactivate
cd ~/Alphabata_ws
colcon build --packages-select AAAmodel
source install/setup.bash
\`\`\`

## ⚠️ 避坑指南 / 注意事项

1. **Conda 冲突**: ROS 2 的底层 C++ 绑定 (`_rclpy_pybind11`) 依赖于系统自带的 Python 3.12。请务必在运行任何 `ros2` 工具前确保 `(base)` 环境已关闭 (`conda deactivate`)。
2. **摄像头权限**: 如果一键启动时驱动节点报错 `Permission denied`，请赋予 USB 摄像头节点读写权限：
   \`\`\`bash
   sudo chmod 777 /dev/video2
   \`\`\`