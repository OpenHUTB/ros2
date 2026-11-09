"""在 ROS 2 环境中验证带行补齐的 RGB 发布、真实推理和结果消息。"""
import json
import signal
import subprocess
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from town10_facade_defect_detection.detector import asset_directory, read_image


def main():
    log = open("/tmp/town10_defect_live_node.log", "w", encoding="utf-8")
    process = subprocess.Popen(["ros2", "run", "town10_facade_defect_detection", "facade_detect",
                                "--ros-args", "-p", "mode:=live"], stdout=log, stderr=subprocess.STDOUT)
    rclpy.init()
    probe = Node("town10_detection_interface_check")
    publisher = probe.create_publisher(Image, "/town10/image_raw", qos_profile_sensor_data)
    received = {}
    annotated_sub = probe.create_subscription(Image, "/town10/defects/image",
                                              lambda msg: received.__setitem__("image", msg), 1)
    result_sub = probe.create_subscription(String, "/town10/defects/detections",
                                           lambda msg: received.__setitem__("json", json.loads(msg.data)), 10)
    try:
        source = sorted((asset_directory() / "samples").glob("*.png"))[0]
        bgr = read_image(source)
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        h, w = rgb.shape[:2]
        rows = np.full((h, w * 3 + 7), 177, dtype=np.uint8)
        rows[:, :w * 3] = rgb.reshape(h, w * 3)
        message = Image()
        message.header.frame_id = "town10_front_camera"
        message.header.stamp.sec, message.header.stamp.nanosec = 12, 345
        message.height, message.width, message.step = h, w, rows.shape[1]
        message.encoding, message.data = "rgb8", rows.tobytes()
        deadline, next_publish = time.monotonic() + 35, 0
        while time.monotonic() < deadline and len(received) < 2:
            rclpy.spin_once(probe, timeout_sec=0.1)
            if time.monotonic() >= next_publish and publisher.get_subscription_count():
                publisher.publish(message)
                next_publish = time.monotonic() + 1
        assert len(received) == 2, "未在期限内收到两种检测输出"
        output, payload = received["image"], received["json"]
        assert (output.width, output.height, output.step, output.encoding) == (w, h, w * 3, "bgr8")
        assert output.header.frame_id == message.header.frame_id
        assert output.header.stamp == message.header.stamp
        assert len(output.data) == h * w * 3
        assert payload["stamp"] == {"sec": 12, "nanosec": 345}
        assert payload["detections"], "样本图像应检出候选区域"
        assert all({"class_id", "class_name", "confidence", "xyxy", "is_defect"} <= set(d)
                   for d in payload["detections"])
        assert payload["n_defects"] == sum(d["class_name"] != "glass" for d in payload["detections"])
        result = {"input_encoding": "rgb8", "input_step": message.step, "output_encoding": output.encoding,
                  "output_step": output.step, "width": w, "height": h, "source_stamp_preserved": True,
                  "frame_id_preserved": True, "detections": len(payload["detections"]),
                  "defects": payload["n_defects"], "status": "passed"}
        Path("/tmp/town10_defect_ros_interface.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
        print(json.dumps(result, indent=2))
    finally:
        probe.destroy_node()
        rclpy.shutdown()
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)
        log.close()


if __name__ == "__main__":
    main()
