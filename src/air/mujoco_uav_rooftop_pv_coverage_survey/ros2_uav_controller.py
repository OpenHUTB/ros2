"""Sensor-driven planar route tracking / 传感器驱动的平面航线跟踪。"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import time

import numpy as np

from rooftop_pv.expert import ExpertConfig, ExpertInput, FlightLineExpert, wrap_angle
from rooftop_pv.models import TinyImageController, TinyMLP, TinyUNet, TinyWaypointPlanner
from rooftop_pv.planning import planning_features


def load_checkpoint(path: Path, model_class):
    """Load named trained parameters; reject unlabelled random state dictionaries."""
    import torch

    payload = torch.load(path, map_location="cpu", weights_only=True)
    if not isinstance(payload, dict) or payload.get("model") != model_class.__name__:
        raise ValueError(f"checkpoint model must be {model_class.__name__}")
    if payload.get("format_version") != 1 or int(payload.get("epochs", 0)) < 1:
        raise ValueError("checkpoint must contain trained model metadata")
    model = model_class()
    model.load_state_dict(payload["state_dict"], strict=True)
    model.eval()
    return model


def predict_command(model: TinyMLP, cross_track: float, heading: float, distance: float) -> tuple[float, float]:
    import torch

    with torch.inference_mode():
        output = model(torch.tensor([[cross_track, heading, distance]], dtype=torch.float32))[0]
    values = float(output[0]), float(output[1])
    if not all(math.isfinite(value) for value in values):
        raise ValueError("control prediction is not finite")
    return values


def predict_waypoint(model: TinyWaypointPlanner, cross_track: float, heading: float, distance: float) -> tuple[float, float]:
    """Infer a local body-frame waypoint, not velocity / 推理机体系局部目标点。"""
    import torch

    with torch.inference_mode():
        output = model(torch.tensor([planning_features(cross_track, heading, distance)], dtype=torch.float32))[0]
    waypoint = float(output[0]), float(output[1])
    if not all(math.isfinite(value) for value in waypoint) or math.hypot(*waypoint) < 1e-6:
        raise ValueError("planning prediction must be finite and nonzero")
    return waypoint


def predict_image_command(model: TinyImageController, rgb: np.ndarray) -> tuple[float, float]:
    """Apply trained image-to-command weights / 使用已训练图像到控制权重。"""
    import torch
    from torch.nn import functional as F

    image = torch.from_numpy(rgb.astype(np.float32).transpose(2, 0, 1))[None] / 255.0
    image = F.interpolate(image, size=(120, 160), mode="bilinear", align_corners=False)
    with torch.inference_mode():
        output = model(image)[0]
    command = float(output[0]), float(output[1])
    if not all(math.isfinite(value) for value in command):
        raise ValueError("image control prediction is not finite")
    return command


def image_rgb(message) -> np.ndarray:
    """Decode common ROS Image layouts, including row padding / 解析带行填充的 ROS 图像。"""
    channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}
    if message.encoding not in channels:
        raise ValueError(f"unsupported image encoding: {message.encoding}")
    count = channels[message.encoding]
    width, height, step = int(message.width), int(message.height), int(message.step)
    if width <= 0 or height <= 0 or step < width * count:
        raise ValueError("invalid image dimensions or row stride")
    data = np.frombuffer(bytes(message.data), dtype=np.uint8)
    if data.size < height * step:
        raise ValueError("image buffer is shorter than height * step")
    array = data[:height * step].reshape(height, step)[:, :width * count]
    array = array.reshape(height, width, count)
    if count == 1:
        return np.repeat(array, 3, axis=2)
    if message.encoding.startswith("bgr"):
        return array[:, :, [2, 1, 0]].copy()
    return array[:, :, :3].copy()


class TrackingCore:
    """ROS-independent safety and route tracking state / 独立于 ROS 的安全与航迹状态。"""

    def __init__(self, *, max_speed=1.0, max_yaw_rate=1.2, waypoint_tolerance=0.5,
                 input_timeout=1.0, control_model=None, segmentation_model=None, planning_model=None,
                 end_to_end_model=None):
        values = (max_speed, max_yaw_rate, waypoint_tolerance, input_timeout)
        if any(not math.isfinite(value) or value <= 0 for value in values):
            raise ValueError("bounds, tolerance and timeout must be finite and positive")
        self.max_speed, self.max_yaw_rate = float(max_speed), float(max_yaw_rate)
        self.tolerance, self.timeout = float(waypoint_tolerance), float(input_timeout)
        self.model, self.segmentation_model = control_model, segmentation_model
        self.planning_model = planning_model
        self.image_model, self.rgb = end_to_end_model, None
        self.pose, self.pose_frame, self.pose_time = None, "", -math.inf
        self.route, self.route_frame, self.target_index = [], "", 0
        self.last_image_time = -math.inf
        self.perception = {"state": "waiting_image", "source": self.perception_source}

    @property
    def perception_source(self):
        return "trained_segmentation" if self.segmentation_model is not None else "color_baseline_no_checkpoint"

    def set_pose(self, x: float, y: float, yaw: float, frame: str, now: float) -> None:
        if not all(math.isfinite(value) for value in (x, y, yaw, now)):
            self.pose = None
            return
        self.pose, self.pose_frame, self.pose_time = (x, y, yaw), frame, now

    def set_route(self, points: list[tuple[float, float]], frame: str) -> None:
        if not points or any(not all(math.isfinite(v) for v in p) for p in points):
            self.route = []
            return
        unique = [points[0]]
        for point in points[1:]:
            if math.hypot(point[0] - unique[-1][0], point[1] - unique[-1][1]) > 1e-6:
                unique.append(point)
        if unique != self.route or frame != self.route_frame:
            self.target_index = min(1, len(unique) - 1)
        self.route, self.route_frame = unique, frame

    def set_image(self, rgb: np.ndarray, now: float) -> None:
        import torch
        from torch.nn import functional as F

        if rgb.ndim != 3 or rgb.shape[2] != 3 or min(rgb.shape[:2]) < 1 or not math.isfinite(now):
            raise ValueError("image must be a nonempty RGB array with a finite reception time")
        if self.segmentation_model is None:
            values = rgb.astype(np.int16)
            mask = (values[:, :, 2] > values[:, :, 0] + 10) & (values[:, :, 2] > values[:, :, 1] + 10)
        else:
            image = torch.from_numpy(rgb.astype(np.float32).transpose(2, 0, 1))[None] / 255.0
            image = F.interpolate(image, size=(120, 160), mode="bilinear", align_corners=False)
            with torch.inference_mode():
                mask = (torch.sigmoid(self.segmentation_model(image))[0, 0] > 0.5).numpy()
        self.last_image_time = now
        self.rgb = rgb.copy()
        self.perception = {"state": "ready", "source": self.perception_source,
                           "pv_pixel_ratio": float(mask.mean()), "pv_pixels": int(mask.sum())}

    def command(self, now: float) -> tuple[float, float, dict]:
        status = {"controller": "trained_image" if self.image_model is not None else
                  ("trained_mlp" if self.model is not None else "geometric_tracking_no_checkpoint"),
                  "planner": "direct_image_policy" if self.image_model is not None else
                  ("trained_local_waypoint" if self.planning_model is not None else "route_geometry"),
                  "perception": dict(self.perception), "target_index": self.target_index}
        if now - self.last_image_time > self.timeout and status["perception"]["state"] == "ready":
            status["perception"]["state"] = "stale_image"
        if self.pose is None:
            return 0.0, 0.0, {**status, "state": "waiting_pose"}
        if now - self.pose_time > self.timeout:
            return 0.0, 0.0, {**status, "state": "stale_pose"}
        if not self.route:
            return 0.0, 0.0, {**status, "state": "waiting_route"}
        if not self.pose_frame or self.pose_frame != self.route_frame:
            return 0.0, 0.0, {**status, "state": "frame_mismatch"}
        x, y, yaw = self.pose
        final = self.route[-1]
        if math.hypot(final[0] - x, final[1] - y) <= self.tolerance:
            return 0.0, 0.0, {**status, "state": "arrived", "target_index": len(self.route) - 1}
        if self.rgb is None or self.perception["state"] != "ready":
            return 0.0, 0.0, {**status, "state": "waiting_image"}
        if now - self.last_image_time > self.timeout:
            return 0.0, 0.0, {**status, "state": "stale_image"}
        while self.target_index < len(self.route) - 1:
            tx, ty = self.route[self.target_index]
            if math.hypot(tx - x, ty - y) > self.tolerance:
                break
            self.target_index += 1
        target = self.route[self.target_index]
        start = self.route[max(0, self.target_index - 1)]
        dx, dy = target[0] - start[0], target[1] - start[1]
        length = math.hypot(dx, dy)
        path_heading = math.atan2(dy, dx) if length > 1e-6 else math.atan2(target[1] - y, target[0] - x)
        cross_track = ((x - start[0]) * -dy + (y - start[1]) * dx) / max(length, 1e-6)
        heading_error = wrap_angle(yaw - path_heading)
        distance = math.hypot(target[0] - x, target[1] - y)
        turn_error = wrap_angle(math.atan2(target[1] - y, target[0] - x) - yaw)
        control_cross_track, control_heading, control_distance = cross_track, heading_error, distance
        if self.planning_model is not None and self.image_model is None:
            along_distance = max(0.0, (target[0] - x) * math.cos(path_heading) +
                                 (target[1] - y) * math.sin(path_heading))
            local_target = predict_waypoint(self.planning_model, cross_track, heading_error, along_distance)
            turn_error = math.atan2(local_target[1], local_target[0])
            control_cross_track, control_heading, control_distance = 0.0, -turn_error, math.hypot(*local_target)
            status["planned_local_waypoint_m"] = list(local_target)
        if self.image_model is not None:
            forward, turn = predict_image_command(self.image_model, self.rgb)
        elif self.model is not None:
            forward, turn = predict_command(self.model, control_cross_track, control_heading, control_distance)
        else:
            forward = self.max_speed * max(0.0, math.cos(turn_error)) * min(1.0, control_distance)
            turn = 1.4 * turn_error
        forward = max(0.0, min(self.max_speed, forward))
        turn = max(-self.max_yaw_rate, min(self.max_yaw_rate, turn))
        if abs(wrap_angle(math.atan2(target[1] - y, target[0] - x) - yaw)) > math.pi / 2:
            forward = 0.0
        return forward, turn, {**status, "state": "tracking", "target_index": self.target_index,
                               "cross_track_error": cross_track, "heading_error": heading_error,
                               "distance_to_target": distance}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="ROS 2 rooftop controller / 屋顶航迹控制节点")
    parser.add_argument("--replay", type=Path, help="offline JSONL replay / 离线 JSONL 回放")
    parser.add_argument("--checkpoint", type=Path, help="trained control.pt for replay / 回放控制模型")
    args, ros_args = parser.parse_known_args(argv)
    if args.replay:
        if ros_args:
            parser.error(f"unrecognized replay arguments: {' '.join(ros_args)}")
        model = load_checkpoint(args.checkpoint, TinyMLP) if args.checkpoint else None
        expert = FlightLineExpert(ExpertConfig())
        for line in args.replay.read_text(encoding="utf-8").splitlines():
            row = json.loads(line)
            state = ExpertInput(row["cross_track_error"], row["heading_error"], row["distance_to_turn"])
            if model is None:
                command = expert.command(state)
                values = command.forward_speed, command.yaw_rate
            else:
                values = predict_command(model, state.cross_track_error, state.heading_error, state.distance_to_turn)
            print(json.dumps({"forward_speed": values[0], "yaw_rate": values[1],
                              "controller": "trained_mlp" if model is not None else "geometric_expert"}))
        return 0
    if args.checkpoint:
        parser.error("--checkpoint requires --replay; use control_checkpoint ROS parameter for live mode")
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from geometry_msgs.msg import PoseStamped, Twist
        from nav_msgs.msg import Path as RoutePath
        from sensor_msgs.msg import Image
        from std_msgs.msg import String
    except ImportError as exc:
        raise SystemExit("ROS 2 Humble dependencies are required for live mode / 实时模式需要 ROS 2 Humble 依赖") from exc

    class Controller(Node):
        def __init__(self):
            super().__init__("rooftop_pv_controller")
            defaults = {
                "image_topic": "/camera/image_raw", "pose_topic": "/uav/pose", "route_topic": "/uav/route",
                "command_topic": "/cmd_vel", "status_topic": "/rooftop_pv/status",
                "control_checkpoint": "", "segmentation_checkpoint": "", "planning_checkpoint": "",
                "end_to_end_checkpoint": "",
                "max_speed": 1.0, "max_yaw_rate": 1.2, "waypoint_tolerance": 0.5,
                "input_timeout": 1.0, "control_period": 0.1,
            }
            for name, default in defaults.items():
                self.declare_parameter(name, default)
            p = {name: self.get_parameter(name).value for name in defaults}
            control = load_checkpoint(Path(p["control_checkpoint"]), TinyMLP) if p["control_checkpoint"] else None
            segmentation = load_checkpoint(Path(p["segmentation_checkpoint"]), TinyUNet) if p["segmentation_checkpoint"] else None
            planning = load_checkpoint(Path(p["planning_checkpoint"]), TinyWaypointPlanner) if p["planning_checkpoint"] else None
            end_to_end = load_checkpoint(Path(p["end_to_end_checkpoint"]), TinyImageController) if p["end_to_end_checkpoint"] else None
            self.core = TrackingCore(**{name: float(p[name]) for name in
                                     ("max_speed", "max_yaw_rate", "waypoint_tolerance", "input_timeout")},
                                     control_model=control, segmentation_model=segmentation, planning_model=planning,
                                     end_to_end_model=end_to_end)
            period = float(p["control_period"])
            if not math.isfinite(period) or period <= 0:
                raise ValueError("control_period must be finite and positive")
            self.publisher = self.create_publisher(Twist, p["command_topic"], 10)
            self.status_publisher = self.create_publisher(String, p["status_topic"], 10)
            self.pose_subscription = self.create_subscription(PoseStamped, p["pose_topic"], self.on_pose, qos_profile_sensor_data)
            self.route_subscription = self.create_subscription(RoutePath, p["route_topic"], self.on_route, 10)
            self.image_subscription = self.create_subscription(Image, p["image_topic"], self.on_image, qos_profile_sensor_data)
            self.timer = self.create_timer(period, self.publish_command)

        def on_pose(self, message):
            p, q = message.pose.position, message.pose.orientation
            norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
            if not math.isfinite(norm) or norm < 1e-8:
                self.core.pose = None
                return
            qx, qy, qz, qw = q.x/norm, q.y/norm, q.z/norm, q.w/norm
            yaw = math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))
            self.core.set_pose(float(p.x), float(p.y), yaw, message.header.frame_id, time.monotonic())

        def on_route(self, message):
            if any(p.header.frame_id and p.header.frame_id != message.header.frame_id for p in message.poses):
                self.core.route = []
                return
            self.core.set_route([(float(p.pose.position.x), float(p.pose.position.y)) for p in message.poses], message.header.frame_id)

        def on_image(self, message):
            try:
                self.core.set_image(image_rgb(message), time.monotonic())
            except (ValueError, RuntimeError) as exc:
                self.core.perception = {"state": "invalid_image", "detail": str(exc), "source": self.core.perception_source}

        def publish_command(self):
            try:
                forward, turn, status = self.core.command(time.monotonic())
            except (ValueError, RuntimeError) as exc:
                forward, turn, status = 0.0, 0.0, {"state": "control_error", "detail": str(exc)}
            command = Twist()
            command.linear.x, command.angular.z = forward, turn
            self.publisher.publish(command)
            report = String()
            report.data = json.dumps(status, sort_keys=True)
            self.status_publisher.publish(report)

    rclpy.init(args=ros_args)
    node = None
    try:
        node = Controller()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
