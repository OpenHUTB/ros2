#!/usr/bin/env python3
import rosbag
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import interp1d
import rospy
import os

def main():
    rospy.init_node("bag_analysis_node")
    bag_path = rospy.get_param("~bag_path", "./robot_run.bag")
    save_dir = rospy.get_param("~save_dir", "./docs/img/")
    os.makedirs(save_dir, exist_ok=True)

    time_cmd = []
    vel_cmd = []
    time_odom = []
    vel_odom = []
    lidar_x = []
    lidar_y = []

    bag = rosbag.Bag(bag_path, 'r')
    for topic, msg, t in bag.read_messages(topics=['/cmd_vel', '/odom', '/scan']):
        t_sec = t.to_sec()
        if topic == "/cmd_vel":
            v = msg.linear.x
            time_cmd.append(t_sec)
            vel_cmd.append(v)
        elif topic == "/odom":
            v = msg.twist.twist.linear.x
            time_odom.append(t_sec)
            vel_odom.append(v)
        elif topic == "/scan":
            ranges = np.array(msg.ranges)
            angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
            valid = np.isfinite(ranges)
            r_valid = ranges[valid]
            ang_valid = angles[valid]
            x = r_valid * np.cos(ang_valid)
            y = r_valid * np.sin(ang_valid)
            lidar_x = x
            lidar_y = y
    bag.close()

    # 图1 指令速度vs实际速度
    plt.figure(figsize=(10,5))
    plt.plot(time_cmd, vel_cmd, label="Command velocity (m/s)")
    plt.plot(time_odom, vel_odom, label="Actual velocity (m/s)")
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (m/s)")
    plt.title("Command vs Actual Velocity")
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.savefig(os.path.join(save_dir, "command_vs_actual_velocity.png"), dpi=300, bbox_inches="tight")
    plt.close()

    # 图2 速度误差
    if len(time_cmd) > 1 and len(time_odom) > 1:
        t_min = max(min(time_cmd), min(time_odom))
        t_max = min(max(time_cmd), max(time_odom))
        t_sample = np.linspace(t_min, t_max, 500)
        f_cmd = interp1d(time_cmd, vel_cmd, kind="linear", fill_value="extrapolate")
        f_odom = interp1d(time_odom, vel_odom, kind="linear", fill_value="extrapolate")
        v_cmd_interp = f_cmd(t_sample)
        v_odom_interp = f_odom(t_sample)
        err = v_cmd_interp - v_odom_interp
        plt.figure(figsize=(10,5))
        plt.plot(t_sample, err, color="#d62728")
        plt.xlabel("Time (s)")
        plt.ylabel("Velocity Error (m/s)")
        plt.title("Velocity Error (cmd - odom)")
        plt.grid(True, alpha=0.3)
        plt.savefig(os.path.join(save_dir, "velocity_error_curve.png"), dpi=300, bbox_inches="tight")
        plt.close()

    # 图3 激光点云
    plt.figure(figsize=(6,6))
    plt.scatter(lidar_x, lidar_y, s=1, c="#1f77b4")
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.title("LiDAR Point Cloud")
    plt.axis("equal")
    plt.grid(True, alpha=0.3)
    plt.savefig(os.path.join(save_dir, "lidar_point_cloud.png"), dpi=300, bbox_inches="tight")
    plt.close()
    rospy.loginfo("✅ 图表保存完成！")

if __name__ == "__main__":
    main()
