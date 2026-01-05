# Livox ROS Driver 2 - Auto-Level Edition

基于 Livox ROS Driver 2 官方驱动的增强版本，新增 **MID360 自动水平校准**功能，支持雷达任意角度安装（倾斜/倒置），自动矫正输出的点云与 IMU 数据为水平坐标系，无需手动测量安装角度。

## 核心特性

### ✨ 自动水平校准（Auto-Level Calibration）

- **问题**：MID360 倾斜/倒置安装时，安装角度难以精确测量
- **解决方案**：
  - 启动时静止采集 IMU 加速度，自动估计 roll/pitch（忽略 yaw）
  - 同时旋转点云与 IMU 输出，保证 faster-lio 等 SLAM 算法正常耦合
  - 可选：自动更新 faster-lio yaml 中的 `extrinsic_T`（IMU-LiDAR 平移外参）
  - 校准结果保存到 `config/MID360_config_calib.json`，后续直接使用

### 📦 兼容原版功能

- 支持 HAP / MID360 / 混合多雷达
- PointCloud2 / CustomMsg / PCL 格式
- ROS1 (Noetic) / ROS2 (Foxy/Humble)

---

## 快速开始

### 1. 安装依赖

#### ROS (推荐 Noetic)
```bash
# Ubuntu 20.04
sudo apt install ros-noetic-desktop-full
```

#### Livox SDK 2
参考官方文档：[Livox-SDK2 安装说明](https://github.com/Livox-SDK/Livox-SDK2)

### 2. 编译

```bash
# ROS1
cd /path/to/your_workspace
source /opt/ros/noetic/setup.bash
catkin_make -DROS_EDITION=ROS1
```

```bash
# ROS2 (Foxy/Humble)
source /opt/ros/foxy/setup.bash
colcon build
```

### 3. 使用

#### 方式 A：一键校准 + 可视化（推荐首次使用）

```bash
# 确保雷达已连接并静止，运行验证脚本
source devel/setup.bash
./src/livox_ros_driver2/scripts/calib_and_visualize.sh
```

该脚本会：
1. 自动校准（3秒静止采样）
2. 生成 `config/MID360_config_calib.json`
3. 启动驱动 + RViz 可视化

#### 方式 B：仅校准（不启动可视化）

```bash
# 需要先启动 roscore 和 livox 驱动（使用原始 config）
roslaunch livox_ros_driver2 msg_MID360.launch

# 新终端运行校准
python3 src/livox_ros_driver2/scripts/mid360_autolevel_calib.py \
  --duration 3.0 \
  --imu-topic /livox/imu \
  --lidar-topic /livox/lidar \
  --fasterlio-yaml /path/to/faster-lio/config/mid360.yaml
```

校准完成后，使用校准配置启动驱动：

```bash
roslaunch livox_ros_driver2 autolevel_MID360.launch
```

#### 方式 C：原版使用方式（不校准）

```bash
# MID360 + PointCloud2 + RViz
roslaunch livox_ros_driver2 rviz_MID360.launch

# MID360 + CustomMsg（faster-lio 建图）
roslaunch livox_ros_driver2 msg_MID360.launch xfer_format:=1
```

---

## 校准参数说明

`mid360_autolevel_calib.py` 命令行参数：

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `--base-config` | 基础配置 JSON 路径 | `config/MID360_config.json` |
| `--duration` | 校准采样时长（秒） | `3.0` |
| `--timeout` | 数据检查超时（秒） | `8.0` |
| `--imu-topic` | IMU topic 名称 | `/livox/imu` |
| `--lidar-topic` | 点云 topic 名称 | `/livox/lidar` |
| `--target-ip` | 指定雷达 IP（多雷达场景） | 空（全部） |
| `--fasterlio-yaml` | faster-lio yaml 路径（可选） | 空（不更新） |

---

## 配置文件说明

### 核心配置参数

#### `config/MID360_config.json`（或自定义路径）

```json
{
  "MID360": {
    "host_net_info": {
      "cmd_data_ip": "192.168.1.102",
      "point_data_ip": "192.168.1.102",
      "imu_data_ip": "192.168.1.102"
    }
  },
  "lidar_configs": [
    {
      "ip": "192.168.1.191",
      "pcl_data_type": 1,
      "pattern_mode": 0,
      "extrinsic_parameter": {
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
        "x": 0,
        "y": 0,
        "z": 0
      }
    }
  ]
}
```

**关键参数**：
- `host_net_info.xxx_data_ip`：主机 IP（需与雷达同网段）
- `lidar_configs[].ip`：雷达 IP
- `extrinsic_parameter`：外参（校准后会自动填充 roll/pitch）

### Launch 文件参数

| 参数 | 说明 | 推荐值 |
|------|------|--------|
| `xfer_format` | 点云格式：`0`=PointCloud2，`1`=CustomMsg | faster-lio 用 `1` |
| `output_type` | 输出方式：`0`=topic，`1`=bag only | 建图用 `0` 或 `1` |
| `multi_topic` | 多雷达独立 topic：`0`=共享，`1`=独立 | 单雷达用 `0` |
| `publish_freq` | 发布频率（Hz） | `10.0` |

---

## 工作原理

### 校准流程

1. **启动临时驱动**：使用"外参清零"的临时配置（raw IMU/点云）
2. **数据检查**：等待点云/IMU topic 有数据（超时则报错）
3. **静止采样**：订阅 IMU，采集 N 个加速度样本（默认 3 秒）
4. **姿态估计**：基于重力方向计算 roll/pitch（公式：`roll=atan2(ay,az), pitch=atan2(-ax,sqrt(ay²+az²))`）
5. **生成配置**：写入 `config/MID360_config_calib.json`（仅改 roll/pitch，保留原始 yaw/xyz）
6. **更新 faster-lio**（可选）：对 `extrinsic_T` 做同步旋转变换
7. **退出**：校准完成，停止临时驱动

### 关键设计

- **点云与 IMU 同步旋转**：驱动已在 `pub_handler.cpp` 中实现"外参旋转同时作用到点云与 IMU 输出"（L161-172），保证 SLAM 算法正常耦合
- **不改 C++ 源码**：仅修改配置文件，兼容官方驱动升级
- **单位兼容**：自动识别 IMU 加速度单位（`1g` 或 `9.81m/s²`）

---

## 常见问题

### Q1：校准时报错 "timeout waiting topic: /livox/imu"
- **原因**：雷达未连接或网络配置错误
- **排查**：
  ```bash
  rostopic list  # 确认 /livox/imu 存在
  rostopic hz /livox/imu  # 确认有数据
  ```

### Q2：校准时报错 "robot not static? gyro_norm: xxx"
- **原因**：机器人在移动
- **解决**：保持机器人静止 5 秒后重新校准

### Q3：校准时报错 "acc norm abnormal: xxx"
- **原因**：IMU 数据异常或单位不匹配
- **排查**：检查 `rostopic echo /livox/imu -n 1`，加速度模长应约为 1 或 9.81

### Q4：faster-lio 建图效果仍然差
- **原因**：可能是 `extrinsic_T` 未更新或雷达盲区设置不当
- **解决**：
  1. 校准时加 `--fasterlio-yaml` 参数自动更新外参
  2. 检查 `faster-lio/config/mid360.yaml` 中 `blind` 参数（推荐 1.5-2.0）

---

## 与官方版本差异

本仓库基于 [Livox-SDK/livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) v1.2.4，新增：

- `scripts/mid360_autolevel_calib.py`：自动水平校准脚本
- `scripts/calib_and_visualize.sh`：一键校准 + 可视化验证
- `launch_ROS1/autolevel_MID360.launch`：使用校准配置的启动文件
- `config/MID360_config_calib.json`：校准输出配置（自动生成）

其余功能与官方版本完全一致。

---

## License

MIT License (与官方版本相同)

## 致谢

- Livox 官方驱动：[livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)
- faster-lio：[faster-lio](https://github.com/gaoxiang12/faster-lio)
