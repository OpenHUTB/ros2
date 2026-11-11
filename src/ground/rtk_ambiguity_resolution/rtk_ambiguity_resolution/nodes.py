"""ROS 2 nodes: a double-difference observation source and an RTK solver."""

from __future__ import annotations

import csv
import json
import pathlib

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .core import decorrelate, fix, float_solution
from .scene import RtkConfig, simulate


class RtkSimulator(Node):
    """Publish one fix window worth of double-difference observations."""

    def __init__(self) -> None:
        super().__init__("rtk_sim")
        self.declare_parameter("window_epochs", 20)
        self.declare_parameter("windows", 0)
        self.declare_parameter("seed", 0)
        self.window_epochs = max(1, int(self.get_parameter("window_epochs").value))
        self.windows = int(self.get_parameter("windows").value)  # 0 = 无限循环
        self.seed = int(self.get_parameter("seed").value)
        config = RtkConfig(epochs=self.window_epochs, epochs_per_fix=self.window_epochs, seed=self.seed)
        self.epochs = simulate(config)
        self.publisher = self.create_publisher(String, "/rtk/observation", 10)
        self.index = 0
        self.published = 0
        self.timer = self.create_timer(0.5, self.step)
        self.get_logger().info(
            f"rtk_sim ready: {len(self.epochs)} epochs of {self.window_epochs} samples"
        )

    def step(self) -> None:
        if self.windows > 0 and self.published >= self.windows:
            self.timer.cancel()
            self.get_logger().info(f"published {self.published} windows, shutting down")
            rclpy.shutdown()
            return
        payload = {
            "design": np.vstack([epoch["design"] for epoch in self.epochs]).tolist(),
            "phase_cycles": np.concatenate(
                [epoch["phase_cycles"] for epoch in self.epochs]
            ).tolist(),
            "code_m": np.concatenate([epoch["code_m"] for epoch in self.epochs]).tolist(),
            "ambiguity_count": int(len(self.epochs[0]["ambiguity"])),
            "truth_baseline": self.epochs[0]["baseline"].tolist(),
            "truth_ambiguity": self.epochs[0]["ambiguity"].tolist(),
        }
        self.publisher.publish(String(data=json.dumps(payload)))
        self.published += 1


class RtkSolver(Node):
    """Float solution, LAMBDA decorrelation and ratio test on each window."""

    def __init__(self) -> None:
        super().__init__("rtk_solver")
        self.declare_parameter("sigma_phase_cycles", 0.01)
        self.declare_parameter("sigma_code_m", 0.30)
        self.declare_parameter("ratio_threshold", 3.0)
        self.declare_parameter("output_csv", "/tmp/rtk_solver.csv")
        self.sigma_phase = float(self.get_parameter("sigma_phase_cycles").value)
        self.sigma_code = float(self.get_parameter("sigma_code_m").value)
        self.threshold = float(self.get_parameter("ratio_threshold").value)
        self.csv_path = pathlib.Path(str(self.get_parameter("output_csv").value))
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        self.csv_file = self.csv_path.open("w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.csv_file, lineterminator="\n")
        self.writer.writerow(["window", "ratio", "fixed", "float_error_mm", "fixed_error_mm"])
        self.windows_done = 0
        self.status_pub = self.create_publisher(String, "/rtk/status", 10)
        self.create_subscription(String, "/rtk/observation", self.on_observation, 10)
        self.get_logger().info(
            f"rtk_solver ready: sigma_phase={self.sigma_phase} cycles, ratio>={self.threshold}"
        )

    def on_observation(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            design = np.asarray(payload["design"], dtype=float)
            phase = np.asarray(payload["phase_cycles"], dtype=float)
            code = np.asarray(payload["code_m"], dtype=float)
            count = int(payload["ambiguity_count"])
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
            self.get_logger().warning(f"ignored malformed observation: {error}")
            return
        solution = float_solution(
            design,
            phase,
            code,
            sigma_phase_cycles=self.sigma_phase,
            sigma_code_m=self.sigma_code,
            ambiguity_count=count,
        )
        result = fix(solution, decorrelate(solution.cov_nn), ratio_threshold=self.threshold)
        truth = np.asarray(payload.get("truth_baseline", [0.0, 0.0, 0.0]), dtype=float)
        status = {
            "float_baseline": solution.baseline.tolist(),
            "fixed_baseline": result.baseline.tolist(),
            "ratio": result.ratio,
            "fixed": bool(result.fixed),
            "integers": result.integer_ambiguity.tolist(),
            "truth_baseline": truth.tolist(),
            "float_error_mm": float(1000.0 * np.linalg.norm(solution.baseline - truth)),
            "fixed_error_mm": float(1000.0 * np.linalg.norm(result.baseline - truth)),
        }
        self.windows_done += 1
        self.writer.writerow(
            [
                self.windows_done,
                f"{status['ratio']:.4f}",
                int(status['fixed']),
                f"{status['float_error_mm']:.4f}",
                f"{status['fixed_error_mm']:.4f}",
            ]
        )
        self.csv_file.flush()
        self.status_pub.publish(String(data=json.dumps(status)))
        self.get_logger().info(
            f"ratio={status['ratio']:.1f} fixed={status['fixed']} "
            f"float={status['float_error_mm']:.1f} mm fixed={status['fixed_error_mm']:.2f} mm"
        )


    def destroy_node(self) -> bool:
        if not self.csv_file.closed:
            self.csv_file.flush()
            self.csv_file.close()
        return super().destroy_node()


def _spin(node: Node) -> None:
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main_rtk_sim(args=None) -> None:
    rclpy.init(args=args)
    _spin(RtkSimulator())


def main_rtk_solver(args=None) -> None:
    rclpy.init(args=args)
    _spin(RtkSolver())
