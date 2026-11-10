"""离线读取 bag 数据，绘制速度对比、速度误差与激光点云图。"""

import os

import matplotlib
matplotlib.use("Agg")  # 无显示环境下直接保存图片

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from rclpy.node import Node

TOPIC_VEL_CMD = "/cmd_vel"
TOPIC_ODOM = "/odom"
TOPIC_SCAN = "/scan"


class BagAnalysisNode(Node):
    """bag 离线分析节点。"""

    def __init__(self):
        super().__init__("bag_analysis_node")
        self.declare_parameter("bag_path", "./robot_run")
        self.declare_parameter("save_dir", "./output")
        self.declare_parameter("max_lidar_points", 200000)

    def run(self):
        """执行完整分析流程，出错时由 main() 统一处理。"""
        bag_path = self.get_parameter("bag_path").value
        save_dir = self.get_parameter("save_dir").value
        if not os.path.exists(bag_path):
            raise RuntimeError(
                f"找不到 bag 路径：{bag_path}。`ros2 bag record -o xxx` 生成的是"
                "目录，可直接把这个目录传给 bag_path；也可指定已有 bag 的"
                "目录或 .bag 文件。")
        os.makedirs(save_dir, exist_ok=True)

        data = self.read_bag(bag_path)
        produced = []
        produced += self.plot_velocity(data, save_dir)
        produced += self.plot_velocity_error(data, save_dir)
        produced += self.plot_lidar(data, save_dir)
        if not produced:
            raise RuntimeError(
                "bag 中没有找到可用数据（/cmd_vel、/odom、/scan），未生成任何图表。")
        self.get_logger().info(
            f"共生成 {len(produced)} 张图，输出目录：{os.path.abspath(save_dir)}")

    def read_bag(self, bag_path):
        """用 rosbags 读取 bag（同时支持 ROS 1 与 ROS 2 格式）。"""
        try:
            from pathlib import Path

            from rosbags.highlevel.anyreader import AnyReader, AnyReaderError
        except ImportError as exc:
            raise RuntimeError(
                "缺少 rosbags 依赖，请执行：pip install rosbags"
                "（国内网络可加 -i https://pypi.tuna.tsinghua.edu.cn/simple，"
                "该源不通时改用官方源 https://pypi.org/simple）") from exc

        # 空目录、普通文件等非法输入会抛 AnyReaderError，统一转成友好提示
        try:
            with AnyReader([Path(bag_path)]) as reader:
                raw = self.parse_bag(reader)
        except AnyReaderError as exc:
            raise RuntimeError(
                f"无法读取 bag：{bag_path}（{exc}）。请传入 "
                "`ros2 bag record -o` 生成的目录，或有效的 .bag 文件。") from exc

        lidar_x, lidar_y = raw["lidar_x"], raw["lidar_y"]
        limit = int(self.get_parameter("max_lidar_points").value)
        if lidar_x:
            xs = np.concatenate(lidar_x)
            ys = np.concatenate(lidar_y)
            if xs.size > limit > 0:
                idx = np.random.default_rng(0).choice(xs.size, limit, replace=False)
                xs, ys = xs[idx], ys[idx]
            lidar_x, lidar_y = xs, ys
        else:
            lidar_x, lidar_y = np.empty(0), np.empty(0)

        self.get_logger().info(
            f"读取完成：/cmd_vel {len(raw['vel_cmd'])} 条，"
            f"/odom {len(raw['vel_odom'])} 条，激光点 {lidar_x.size} 个")
        return {
            "time_cmd": np.asarray(raw["time_cmd"]),
            "vel_cmd": np.asarray(raw["vel_cmd"]),
            "time_odom": np.asarray(raw["time_odom"]),
            "vel_odom": np.asarray(raw["vel_odom"]),
            "lidar_x": lidar_x,
            "lidar_y": lidar_y,
        }

    def parse_bag(self, reader):
        """遍历 bag 中指定话题的消息，按话题累积时间序列。"""
        time_cmd, vel_cmd = [], []
        time_odom, vel_odom = [], []
        lidar_x, lidar_y = [], []

        wanted = (TOPIC_VEL_CMD, TOPIC_ODOM, TOPIC_SCAN)
        connections = [c for c in reader.connections if c.topic in wanted]
        if not connections:
            raise RuntimeError(
                f"bag 中不包含 {', '.join(wanted)} 任一话题，无法分析。")
        start_ns = None
        for connection, timestamp, rawdata in reader.messages(
                connections=connections):
            if start_ns is None:
                start_ns = timestamp
            stamp = (timestamp - start_ns) * 1e-9
            msg = reader.deserialize(rawdata, connection.msgtype)
            if connection.topic == TOPIC_VEL_CMD:
                time_cmd.append(stamp)
                vel_cmd.append(msg.linear.x)
            elif connection.topic == TOPIC_ODOM:
                time_odom.append(stamp)
                vel_odom.append(msg.twist.twist.linear.x)
            else:
                # 关键修复：点云按帧累积，而不是每帧覆盖
                ranges = np.asarray(msg.ranges, dtype=np.float64)
                if ranges.size == 0:
                    continue
                angles = np.linspace(
                    msg.angle_min, msg.angle_max, ranges.size)
                valid = (np.isfinite(ranges)
                         & (ranges >= msg.range_min)
                         & (ranges <= msg.range_max))
                if not valid.any():
                    continue
                lidar_x.append(ranges[valid] * np.cos(angles[valid]))
                lidar_y.append(ranges[valid] * np.sin(angles[valid]))

        return {
            "time_cmd": time_cmd,
            "vel_cmd": vel_cmd,
            "time_odom": time_odom,
            "vel_odom": vel_odom,
            "lidar_x": lidar_x,
            "lidar_y": lidar_y,
        }

    def save_figure(self, fig, path):
        fig.savefig(path, dpi=300, bbox_inches="tight")
        plt.close(fig)
        self.get_logger().info(f"已保存：{path}")
        return [path]

    def plot_velocity(self, data, save_dir):
        if data["time_cmd"].size == 0 or data["time_odom"].size == 0:
            self.get_logger().warning("/cmd_vel 或 /odom 无数据，跳过速度对比图")
            return []
        fig, ax = plt.subplots(figsize=(10, 5))
        ax.plot(data["time_cmd"], data["vel_cmd"], label="Command velocity")
        ax.plot(data["time_odom"], data["vel_odom"], label="Actual velocity")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Velocity (m/s)")
        ax.set_title("Command vs Actual Velocity")
        ax.legend()
        ax.grid(True, alpha=0.3)
        return self.save_figure(
            fig, os.path.join(save_dir, "command_vs_actual_velocity.png"))

    def plot_velocity_error(self, data, save_dir):
        t_cmd, v_cmd = data["time_cmd"], data["vel_cmd"]
        t_odom, v_odom = data["time_odom"], data["vel_odom"]
        if t_cmd.size < 2 or t_odom.size < 2:
            self.get_logger().warning(
                "/cmd_vel 或 /odom 数据不足两条，跳过速度误差图")
            return []
        t_min = max(t_cmd.min(), t_odom.min())
        t_max = min(t_cmd.max(), t_odom.max())
        if t_max <= t_min:
            self.get_logger().warning("两个话题时间区间不重叠，跳过速度误差图")
            return []
        t_sample = np.linspace(t_min, t_max, 500)
        err = (np.interp(t_sample, t_cmd, v_cmd)
               - np.interp(t_sample, t_odom, v_odom))
        fig, ax = plt.subplots(figsize=(10, 5))
        ax.plot(t_sample, err, color="#d62728")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Velocity Error (m/s)")
        ax.set_title("Velocity Error (cmd - odom)")
        ax.grid(True, alpha=0.3)
        return self.save_figure(
            fig, os.path.join(save_dir, "velocity_error_curve.png"))

    def plot_lidar(self, data, save_dir):
        if data["lidar_x"].size == 0:
            self.get_logger().warning("/scan 无有效数据，跳过点云图")
            return []
        fig, ax = plt.subplots(figsize=(6, 6))
        ax.scatter(data["lidar_x"], data["lidar_y"], s=1, c="#1f77b4")
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_title("LiDAR Point Cloud")
        ax.axis("equal")
        ax.grid(True, alpha=0.3)
        return self.save_figure(
            fig, os.path.join(save_dir, "lidar_point_cloud.png"))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = BagAnalysisNode()
        node.run()
    except KeyboardInterrupt:
        pass
    except (RuntimeError, OSError, ValueError) as exc:
        rclpy.logging.get_logger("bag_analysis_node").error(str(exc))
        raise SystemExit(1)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
