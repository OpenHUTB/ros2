"""Exercise installed ROS subscriptions and safety transitions / 验证安装后的 ROS 输入与安全状态。"""

import argparse
import json
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path as RoutePath
from sensor_msgs.msg import Image
from std_msgs.msg import String


def verify(output: Path, *, trained: bool = False) -> dict:
    output.mkdir(parents=True, exist_ok=True)
    package = "mujoco_uav_rooftop_pv_coverage_survey"
    command = ["ros2", "launch", package, "rooftop_pv_controller.launch.py"]
    if trained:
        command = ["ros2", "run", package, "rooftop_pv_controller", "--ros-args",
                   "-p", f"control_checkpoint:={output / 'day2/control.pt'}",
                   "-p", f"segmentation_checkpoint:={output / 'day2/segmentation.pt'}",
                   "-p", f"planning_checkpoint:={output / 'day2/planning.pt'}"]
    label = "trained" if trained else "launch"
    log = (output / f"ros_{label}.log").open("w", encoding="utf-8")
    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
    rclpy.init()
    node = rclpy.create_node("rooftop_runtime_acceptance")
    commands, reports = [], []
    node.create_subscription(Twist, "/cmd_vel", lambda msg: commands.append((msg.linear.x, msg.angular.z)), 10)
    node.create_subscription(String, "/rooftop_pv/status", lambda msg: reports.append(json.loads(msg.data)), 10)
    pose_pub = node.create_publisher(PoseStamped, "/uav/pose", 10)
    route_pub = node.create_publisher(RoutePath, "/uav/route", 10)
    image_pub = node.create_publisher(Image, "/camera/image_raw", 10)

    def spin_until(predicate, timeout=15.0, publish=None):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if process.poll() is not None:
                raise RuntimeError(f"controller exited with {process.returncode}; see ros_{label}.log")
            if publish is not None:
                publish()
            rclpy.spin_once(node, timeout_sec=0.05)
            if predicate():
                return
        raise TimeoutError(f"ROS acceptance timed out; last status: {reports[-1:]}")

    try:
        spin_until(lambda: bool(commands) and bool(reports))
        assert commands[-1] == (0.0, 0.0)
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.orientation.w = 1.0
        pose.pose.position.y = 1.0
        route = RoutePath()
        route.header.frame_id = "map"
        for x in (0.0, 10.0):
            point = PoseStamped()
            point.header.frame_id = "map"
            point.pose.position.x = x
            point.pose.orientation.w = 1.0
            route.poses.append(point)
        image = Image()
        image.height, image.width, image.step, image.encoding = 24, 32, 96, "rgb8"
        image.data = bytes([0, 0, 255] * (24 * 32))

        def publish():
            pose.header.stamp = node.get_clock().now().to_msg()
            pose_pub.publish(pose)
            route_pub.publish(route)
            image_pub.publish(image)

        spin_until(lambda: any(r.get("state") == "tracking" and r.get("perception", {}).get("state") == "ready"
                               for r in reports), publish=publish)
        tracking = next(r for r in reversed(reports) if r.get("state") == "tracking")
        assert any(forward > 0 for forward, _ in commands)
        assert all(0 <= forward <= 1.0 and abs(turn) <= 1.2 for forward, turn in commands)
        if not trained:
            assert any(turn < 0 for _, turn in commands)
            assert tracking["perception"]["pv_pixel_ratio"] == 1.0
        else:
            assert tracking["controller"] == "trained_mlp"
            assert tracking["perception"]["source"] == "trained_segmentation"
        spin_until(lambda: reports[-1].get("state") == "stale_pose" and commands[-1] == (0.0, 0.0), timeout=5.0)
        result = {"passed": True, "mode": label, "samples": len(commands), "tracking": tracking,
                  "timeout_stop": True, "installed_launch": not trained}
        (output / f"ros_{label}_acceptance.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
        return result
    finally:
        node.destroy_node()
        rclpy.shutdown()
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            process.terminate()
            process.wait(timeout=5)
        log.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--trained", action="store_true")
    args = parser.parse_args()
    print(json.dumps(verify(args.output, trained=args.trained), sort_keys=True))
