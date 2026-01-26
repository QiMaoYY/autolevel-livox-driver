#!/bin/bash
set -e
# ============== 配置 ==============
CONDA_ENV="demo"
SLAM_WS="/media/data/slam_ws"

# 1. 激活conda环境
echo "激活conda环境: ${CONDA_ENV}"
source "${HOME}/miniconda3/etc/profile.d/conda.sh"
conda activate ${CONDA_ENV}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SLAM_WS}/livox_ws/devel/setup.bash"
echo "====== Step 1: Running MID360 auto-level calibration ======"
python3 "${SLAM_WS}/livox_ws/src/livox_ros_driver2/scripts/mid360_autolevel_calib.py" \
    --duration 3.0 \
    --timeout 10.0 \
    --imu-topic /livox/imu \
    --lidar-topic /livox/lidar \
    --fasterlio-yaml "${SLAM_WS}/src/faster-lio/config/" \
    "${@}"

echo ""
echo "====== Step 2: Starting livox driver with calibrated config ======"
roslaunch livox_ros_driver2 autolevel_MID360.launch xfer_format:=0 rviz_enable:=false

