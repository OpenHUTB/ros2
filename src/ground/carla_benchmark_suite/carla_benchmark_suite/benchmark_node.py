#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""CARLA 作业综合整合与性能评价 —— ROS 2 Humble 节点。

两种工作方式：

  1. **调度模式**：启动时按 `target` 参数拉起对应子功能包（`control` / `perception` /
     `navigation` / `end_to_end`），把该子包作为独立进程运行。
  2. **评测模式**：把基准评测结果（各模块精度、时延、平滑度）作为 ROS 话题发布，
     便于其它节点订阅或记录到 rosbag。

发布：  /carla/benchmark/metrics        (std_msgs/Float32MultiArray)
        /carla/benchmark/metric_names   (std_msgs/String, 逗号分隔的指标名)

启动：
    ros2 launch carla_benchmark_suite main.launch.py target:=perception
    ros2 launch carla_benchmark_suite main.launch.py metrics_source:=file metrics_file:=report.json
"""

import json
import os
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

_HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)


class BenchmarkNode(Node):
    """评测指标发布节点 / 子模块调度节点。"""

    #: 会把哪些指标发布到话题上（只挑数值型、有物理意义的）
    PUBLISHED_KEYS = [
        "percept_acc", "control_mse", "lateral_rmse",
        "plan_mse", "known_ratio", "occupied_cells", "nav_min_dist",
        "reg_mae", "reg_sign_acc", "reg_corr", "latency_mean_ms",
    ]

    def __init__(self):
        super().__init__("carla_benchmark_node")

        self.declare_parameter("target", "")            # 调度模式：要拉起哪个子模块
        self.declare_parameter("metrics_file", "")      # 评测模式：已导出的 report.json
        self.declare_parameter("publish_period", 5.0)

        self.target = self.get_parameter("target").value
        self.metrics_file = self.get_parameter("metrics_file").value
        period = float(self.get_parameter("publish_period").value)

        self.pub_vals = self.create_publisher(Float32MultiArray, "/carla/benchmark/metrics", 10)
        self.pub_names = self.create_publisher(String, "/carla/benchmark/metric_names", 10)

        self.results = self._load_or_run()

        if self.results:
            keys, vals = self._flatten()
            self.pub_names.publish(String(data=",".join(keys)))
            self.get_logger().info(f"已就绪：将周期性发布 {len(keys)} 项评测指标")
            self._keys, self._vals = keys, vals
            self.create_timer(period, self._on_timer)
        else:
            self.get_logger().warn(
                "未获得评测结果（既未提供 metrics_file，也未在本节点内运行评测）。"
                "可先离线运行： python3 main.py --benchmark --out report.json")

    # ------------------------------------------------------------------ 数据
    def _load_or_run(self):
        """优先读已导出的 JSON；否则在本节点内现场跑一次评测。"""
        if self.metrics_file and os.path.isfile(self.metrics_file):
            try:
                with open(self.metrics_file, encoding="utf-8") as f:
                    obj = json.load(f)
                self.get_logger().info(f"已载入评测结果：{self.metrics_file}")
                return obj.get("results", {})
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"读取 {self.metrics_file} 失败({exc})，改为现场评测")

        which = [self.target] if self.target in ("perception", "navigation", "end_to_end") else None
        try:
            import importlib.util
            spec = importlib.util.spec_from_file_location(
                "bench_main", os.path.join(_HERE, "main.py"))
            mod = importlib.util.module_from_spec(spec)
            sys.modules["bench_main"] = mod
            spec.loader.exec_module(mod)
            self.get_logger().info("现场运行基准评测（无需 CARLA 服务端）...")
            out = {}
            for key in (which or ["perception", "navigation", "end_to_end"]):
                try:
                    out[key] = mod.BENCHMARKS[key](epochs=30)
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().warn(f"评测 {key} 失败：{exc}")
            return out
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"基准评测模块加载失败：{exc}")
            return {}

    def _flatten(self):
        """把嵌套结果压平成 (指标名列表, 数值列表)，跳过非数值项。"""
        keys, vals = [], []
        for mod_name, m in self.results.items():
            if not isinstance(m, dict):
                continue
            for k in self.PUBLISHED_KEYS:
                v = m.get(k)
                if isinstance(v, (int, float)) and not isinstance(v, bool):
                    keys.append(f"{mod_name}.{k}")
                    vals.append(float(v))
        return keys, vals

    # ------------------------------------------------------------------ 定时
    def _on_timer(self):
        self.pub_vals.publish(Float32MultiArray(data=self._vals))


def main(args=None):
    rclpy.init(args=args)
    node = BenchmarkNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
