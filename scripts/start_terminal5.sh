#!/usr/bin/env bash

set -euo pipefail

# 1. 確保退出 conda 環境（防止 python 庫衝突）
conda deactivate || true

# 2. 引入系統 ROS 2 環境與視覺工作空間環境
source /opt/ros/jazzy/setup.bash
source /home/tao/Alphabata_ws/install/setup.bash

# 3. 賦予相機權限（預防 Permission denied 錯誤）
sudo chmod 777 /dev/video2 2>/dev/null || true

log "正在一鍵啟動 AI 戰術決策與 YOLO 視覺系統..."

# 4. 使用 exec 啟動 Python 包的總 launch 文件
exec ros2 launch AAAmodel start_game.launch.py