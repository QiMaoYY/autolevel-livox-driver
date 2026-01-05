#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

echo "====== Step 1: Running MID360 auto-level calibration ======"
rosrun livox_ros_driver2 mid360_autolevel_calib.py \
    --duration 3.0 \
    --timeout 10.0 \
    --imu-topic /livox/imu \
    --lidar-topic /livox/lidar \
    "${@}"

echo ""
echo "====== Step 2: Starting livox driver with calibrated config ======"
roslaunch livox_ros_driver2 autolevel_MID360.launch rviz_enable:=true

