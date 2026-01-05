#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MID360 Auto-level calibration (ROS1)
"""
import argparse
import json
import math
import os
import re
import shutil
import sys
import time

import rospy
import roslaunch
import rospkg

from sensor_msgs.msg import Imu
from sensor_msgs.msg import PointCloud2

try:
    from livox_ros_driver2.msg import CustomMsg
except Exception:
    CustomMsg = None


def _deg(rad):
    return rad * 180.0 / math.pi


def _norm3(x, y, z):
    return math.sqrt(x * x + y * y + z * z)


def _wait_for_topic_message(topic, msg_type, timeout_s):
    start = time.time()
    while not rospy.is_shutdown():
        left = timeout_s - (time.time() - start)
        if left <= 0.0:
            raise TimeoutError("timeout waiting topic: %s" % topic)
        try:
            return rospy.wait_for_message(topic, msg_type, timeout=left)
        except rospy.ROSException:
            continue


def _load_json(path):
    with open(path, "r") as f:
        return json.load(f)


def _dump_json(path, obj):
    d = os.path.dirname(path)
    if d and not os.path.exists(d):
        os.makedirs(d, exist_ok=True)
    with open(path, "w") as f:
        json.dump(obj, f, indent=2, sort_keys=False)


def _calc_rotation_matrix_deg_rp(roll_deg, pitch_deg):
    roll = float(roll_deg) * math.pi / 180.0
    pitch = float(pitch_deg) * math.pi / 180.0
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    return [
        [cp, sr * sp, cr * sp],
        [0.0, cr, -sr],
        [-sp, sr * cp, cr * cp],
    ]


def _mat3_mul_vec3(R, v):
    return [
        R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2],
        R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2],
        R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2],
    ]


def _update_fasterlio_yaml_extrinsic_t(yaml_path, roll_deg, pitch_deg, backup=True):
    if not yaml_path:
        return False
    if not os.path.exists(yaml_path):
        raise RuntimeError("faster-lio yaml not found: %s" % yaml_path)

    with open(yaml_path, "r") as f:
        raw_lines = f.readlines()

    lines = []
    for line in raw_lines:
        if "\\n" in line:
            parts = line.split("\\n")
            for j, p in enumerate(parts):
                if j < len(parts) - 1:
                    lines.append(p + "\n")
                else:
                    if p != "":
                        lines.append(p)
        else:
            lines.append(line)

    idx = None
    for i, line in enumerate(lines):
        if re.match(r"^\s*extrinsic_T\s*:\s*\[", line):
            idx = i
            break
    if idx is None:
        raise RuntimeError("cannot find 'extrinsic_T:' in %s" % yaml_path)

    m = re.search(r"\[\s*([^\]]+)\s*\]", lines[idx])
    if not m:
        raise RuntimeError("failed to parse extrinsic_T list in %s" % yaml_path)
    raw_items = [x.strip() for x in m.group(1).split(",")]
    if len(raw_items) < 3:
        raise RuntimeError("extrinsic_T must have 3 values in %s" % yaml_path)
    t_factory = [float(raw_items[0]), float(raw_items[1]), float(raw_items[2])]

    R = _calc_rotation_matrix_deg_rp(roll_deg, pitch_deg)
    t_new = _mat3_mul_vec3(R, t_factory)

    indent = re.match(r"^(\s*)", lines[idx]).group(1)
    factory_comment = indent + "# factory_extrinsic_T: [%.6f, %.6f, %.6f]\n" % (t_factory[0], t_factory[1], t_factory[2])
    has_factory_comment = (idx - 1 >= 0) and ("factory_extrinsic_T" in lines[idx - 1])

    if backup:
        shutil.copyfile(yaml_path, yaml_path + ".bak")

    if not has_factory_comment:
        lines.insert(idx, factory_comment)
        idx += 1

    lines[idx] = indent + "extrinsic_T: [%.6f, %.6f, %.6f]\n" % (t_new[0], t_new[1], t_new[2])

    with open(yaml_path, "w") as f:
        f.writelines(lines)
    return True


def _make_raw_config(base_cfg, lidar_ip=None):
    cfg = json.loads(json.dumps(base_cfg))
    if "lidar_configs" not in cfg or not cfg["lidar_configs"]:
        raise RuntimeError("invalid config: missing lidar_configs")

    for lc in cfg["lidar_configs"]:
        if lidar_ip is not None and lc.get("ip") != lidar_ip:
            continue
        ext = lc.get("extrinsic_parameter", {})
        ext["roll"] = 0.0
        ext["pitch"] = 0.0
        ext["yaw"] = 0.0
        lc["extrinsic_parameter"] = ext
        if lidar_ip is not None:
            break
    return cfg


def _apply_calib_to_config(base_cfg, roll_deg, pitch_deg, lidar_ip=None):
    cfg = json.loads(json.dumps(base_cfg))
    if "lidar_configs" not in cfg or not cfg["lidar_configs"]:
        raise RuntimeError("invalid config: missing lidar_configs")

    found = False
    for lc in cfg["lidar_configs"]:
        if lidar_ip is not None and lc.get("ip") != lidar_ip:
            continue
        ext = lc.get("extrinsic_parameter", {})
        ext["roll"] = float(roll_deg)
        ext["pitch"] = float(pitch_deg)
        lc["extrinsic_parameter"] = ext
        found = True
        if lidar_ip is not None:
            break

    if not found:
        raise RuntimeError("cannot find lidar ip=%s in lidar_configs" % str(lidar_ip))
    return cfg


class _ImuAccumulator(object):
    def __init__(self):
        self.n = 0
        self.sum_ax = 0.0
        self.sum_ay = 0.0
        self.sum_az = 0.0
        self.sum_gx = 0.0
        self.sum_gy = 0.0
        self.sum_gz = 0.0
        self.sum_norm_g = 0.0
        self.sum_norm_a = 0.0

    def push(self, msg):
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z
        self.n += 1
        self.sum_ax += ax
        self.sum_ay += ay
        self.sum_az += az
        self.sum_gx += gx
        self.sum_gy += gy
        self.sum_gz += gz
        self.sum_norm_g += _norm3(gx, gy, gz)
        self.sum_norm_a += _norm3(ax, ay, az)

    def mean(self):
        if self.n <= 0:
            raise RuntimeError("no imu samples")
        inv = 1.0 / float(self.n)
        return {
            "ax": self.sum_ax * inv,
            "ay": self.sum_ay * inv,
            "az": self.sum_az * inv,
            "gx": self.sum_gx * inv,
            "gy": self.sum_gy * inv,
            "gz": self.sum_gz * inv,
            "gyro_norm": self.sum_norm_g * inv,
            "acc_norm": self.sum_norm_a * inv,
            "n": self.n,
        }


def main():
    parser = argparse.ArgumentParser(description="MID360 auto-level calibration")
    parser.add_argument("--base-config", type=str, default="", help="Base config json path")
    parser.add_argument("--imu-topic", type=str, default="/livox/imu", help="IMU topic")
    parser.add_argument("--lidar-topic", type=str, default="/livox/lidar", help="Lidar topic")
    parser.add_argument("--duration", type=float, default=3.0, help="Calibration duration (s)")
    parser.add_argument("--timeout", type=float, default=8.0, help="Data check timeout (s)")
    parser.add_argument("--target-ip", type=str, default="", help="Target lidar IP")
    parser.add_argument("--fasterlio-yaml", type=str, default="", help="faster-lio yaml to update")
    args = parser.parse_args()

    rospy.init_node("mid360_autolevel_calib", anonymous=False)

    pkg_path = rospkg.RosPack().get_path("livox_ros_driver2")
    base_cfg_path = args.base_config if args.base_config else os.path.join(pkg_path, "config", "MID360_config.json")
    output_cfg_path = os.path.join(pkg_path, "config", "MID360_config_calib.json")
    temp_raw_cfg_path = "/tmp/MID360_config_raw_extrinsic0.json"
    
    target_ip = args.target_ip if args.target_ip else None
    
    rospy.loginfo("Base config: %s", base_cfg_path)
    rospy.loginfo("Output config: %s", output_cfg_path)
    
    base_cfg = _load_json(base_cfg_path)

    # Prepare raw config (extrinsic RPY=0) for calibration
    raw_cfg = _make_raw_config(base_cfg, lidar_ip=target_ip)
    _dump_json(temp_raw_cfg_path, raw_cfg)

    # Start livox driver temporarily with raw config
    rospy.loginfo("Starting livox driver for calibration...")
    rospy.set_param("/user_config_path", temp_raw_cfg_path)
    rospy.set_param("/output_data_type", 0)  # force publish to topics

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()

    node = roslaunch.core.Node(
        package="livox_ros_driver2",
        node_type="livox_ros_driver2_node",
        name="livox_lidar_publisher2",
        output="log",
        respawn=False,
    )
    proc = launch.launch(node)
    if not proc or not proc.is_alive():
        raise RuntimeError("failed to start livox driver")

    try:
        # Check data OK
        rospy.loginfo("Checking topics: lidar=%s imu=%s", args.lidar_topic, args.imu_topic)
        xfer_format = int(rospy.get_param("/xfer_format", 0))
        if xfer_format == 0:
            _wait_for_topic_message(args.lidar_topic, PointCloud2, args.timeout)
        else:
            if CustomMsg is None:
                raise RuntimeError("xfer_format != 0 but CustomMsg not available")
            _wait_for_topic_message(args.lidar_topic, CustomMsg, args.timeout)
        _wait_for_topic_message(args.imu_topic, Imu, args.timeout)
        rospy.loginfo("Topics OK")

        # Calibrate
        rospy.loginfo("Calibrating... keep robot static for %.2f seconds", args.duration)
        acc = _ImuAccumulator()

        def cb(msg):
            acc.push(msg)

        sub = rospy.Subscriber(args.imu_topic, Imu, cb, queue_size=2000)
        try:
            t0 = time.time()
            rate = rospy.Rate(200)
            while not rospy.is_shutdown() and (time.time() - t0) < args.duration:
                rate.sleep()
            m = acc.mean()
        finally:
            sub.unregister()

        if m["n"] < 200:
            raise RuntimeError("imu samples too few: %d < 200" % m["n"])
        if m["gyro_norm"] > 0.15:
            raise RuntimeError("robot not static? gyro_norm: %.4f rad/s" % m["gyro_norm"])
        
        acc_norm = m["acc_norm"]
        ok_g = abs(acc_norm - 1.0) <= 0.5
        ok_ms2 = abs(acc_norm - 9.81) <= 4.0
        if not (ok_g or ok_ms2):
            raise RuntimeError("acc norm abnormal: %.3f" % acc_norm)

        ax, ay, az = m["ax"], m["ay"], m["az"]
        roll = math.atan2(ay, az)
        pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))
        roll_deg = _deg(roll)
        pitch_deg = _deg(pitch)
        
        rospy.loginfo("Calibration done: roll=%.3f deg, pitch=%.3f deg (n=%d, acc_norm=%.3f)",
                      roll_deg, pitch_deg, m["n"], acc_norm)

        # Write calibrated config
        out_cfg = _apply_calib_to_config(base_cfg, roll_deg, pitch_deg, lidar_ip=target_ip)
        _dump_json(output_cfg_path, out_cfg)
        rospy.loginfo("Wrote calibrated config: %s", output_cfg_path)

        # Optionally update faster-lio yaml
        if args.fasterlio_yaml:
            rospy.loginfo("Updating faster-lio yaml: %s", args.fasterlio_yaml)
            _update_fasterlio_yaml_extrinsic_t(args.fasterlio_yaml, roll_deg, pitch_deg, backup=True)
            rospy.loginfo("Updated faster-lio yaml")

    finally:
        # Stop driver
        try:
            proc.stop()
        except Exception:
            pass
        try:
            launch.shutdown()
        except Exception:
            pass

    rospy.loginfo("Calibration finished.")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        rospy.logerr("Calibration failed: %s", str(e))
        sys.exit(1)
