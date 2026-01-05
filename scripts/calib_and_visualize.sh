#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="/media/data/livox_ws"
source $PKG_DIR/devel/setup.bash
echo "====== Step 1: Running MID360 auto-level calibration ======"
python3 $PKG_DIR/src/livox_ros_driver2/scripts/mid360_autolevel_calib.py \
    --duration 3.0 \
    --timeout 10.0 \
    --imu-topic /livox/imu \
    --lidar-topic /livox/lidar \
    "${@}"

echo ""
echo "====== Step 2: Starting livox driver with calibrated config ======"
roslaunch livox_ros_driver2 autolevel_MID360.launch xfer_format:=0 rviz_enable:=true

