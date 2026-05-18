#!/bin/bash
# Alphabata_ws 一键启动脚本
# 请在 ~/Alphabata_ws 目录下运行此脚本

echo "🚀 正在初始化系统环境..."

# 1. 退出 Conda 环境 (防止与 ROS 2 的 Python 版本冲突)
if command -v conda &> /dev/null; then
    eval "$(conda shell.bash hook)"
    conda deactivate
    echo "✅ Conda 环境已退出"
fi

# 2. 加载 ROS 2 系统环境变量 (Jazzy)
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
    echo "✅ ROS 2 Jazzy 环境已加载"
else
    echo "❌ 找不到 ROS 2 Jazzy 环境，请检查安装路径！"
    exit 1
fi

# 3. 加载当前工作空间环境变量
WORKSPACE_DIR="$(pwd)"
if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    source "$WORKSPACE_DIR/install/setup.bash"
    echo "✅ 工作空间 AAAmodel 已加载"
else
    echo "❌ 找不到工作空间环境变量，请先执行 colcon build！"
    exit 1
fi

# 4. 一键启动 Launch 文件
echo "🎮 正在启动视觉与战术 AI 系统..."
# 调用 share/AAAmodel/launch/ 目录下的 start_game.launch.py
ros2 launch AAAmodel start_game.launch.py