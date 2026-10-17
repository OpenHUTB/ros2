#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""车道线检测 ROS 节点：订阅车载相机图像，发布横向偏差等感知结果。

话题接口
--------
  订阅  /carla/ego_vehicle/rgb_front/image   sensor_msgs/Image    前视相机（BGR8/RGB8）
  发布  lane/lateral_offset                  std_msgs/Float32     横向偏差（米，正=车辆偏右）
  发布  lane/heading_error                   std_msgs/Float32     航向偏差（弧度）
  发布  lane/curvature                       std_msgs/Float32     车道曲率半径（米，直道为 inf）
  发布  lane/confidence                       std_msgs/Float32     置信度 0~1
  发布  lane/departure                       std_msgs/Bool        是否压线/偏出阈值
  发布  lane/mask                            sensor_msgs/Image    鸟瞰二值掩码（调试）

参数
----
  见 config/lane_params.yaml，全部参数名与 lane_detection.LaneFitConfig 字段一一对应。

设计说明
--------
检测算法（lane_detection.py）是**纯 numpy** 实现，不依赖 OpenCV；
本节点只做 ROS 收发与消息转换，因此逻辑清晰、易于单测。
图像转换优先用 cv_bridge（快），没有时退回纯 numpy 实现。
"""

import os
import sys

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32

# 允许直接以脚本方式运行（不依赖 catkin 安装）
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from lane_detection import (  # noqa: E402
    LaneDetector,
    LaneFitConfig,
    to_grayscale,
)

# cv_bridge 可选：装了就用（更快、编码处理更周全），没有就走纯 numpy 路径
try:
    from cv_bridge import CvBridge  # noqa: E402

    _HAVE_CV_BRIDGE = True
except ImportError:  # pragma: no cover - 取决于环境
    _HAVE_CV_BRIDGE = False


def _rows_from_buffer(buf, height, width, step, channels):
    """把一维缓冲按行切出 (height, width*channels) 的有效像素。

    真实相机消息常带**行填充**（step > width*channels），必须按 step 切行；
    但也有实现直接令 step == width*channels。这里两种都兼容：
    先尝试按 step 切，若长度不匹配则退化为连续排列。
    """
    expected_padded = height * step
    expected_packed = height * width * channels

    if buf.size == expected_padded and step >= width * channels:
        flat = buf.reshape(height, step)[:, : width * channels]
    elif buf.size == expected_packed:
        flat = buf.reshape(height, width * channels)
    else:
        raise ValueError(
            f"图像数据长度 {buf.size} 与尺寸不匹配"
            f"（height={height}, width={width}, step={step}, channels={channels}）"
        )
    return flat.reshape(height, width, channels)


def image_to_bgr(msg, bridge=None):
    """sensor_msgs/Image -> BGR uint8 数组 (H, W, 3)。

    支持 bgr8 / rgb8 / bgra8 / rgba8 / mono8 编码。
    优先走 cv_bridge；失败或未安装时用纯 numpy 解析 msg.data。
    """
    if bridge is not None:
        try:
            return bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:  # noqa: BLE001 - 退回到手动解码
            pass

    encoding = msg.encoding.lower()
    height, width, step = msg.height, msg.width, msg.step
    buf = np.frombuffer(msg.data, dtype=np.uint8)

    if encoding == "bgr8":
        return np.ascontiguousarray(_rows_from_buffer(buf, height, width, step, 3))

    if encoding == "rgb8":
        arr = _rows_from_buffer(buf, height, width, step, 3)
        return np.ascontiguousarray(arr[:, :, ::-1])

    if encoding == "bgra8":
        arr = _rows_from_buffer(buf, height, width, step, 4)
        return np.ascontiguousarray(arr[:, :, :3])

    if encoding == "rgba8":
        arr = _rows_from_buffer(buf, height, width, step, 4)
        return np.ascontiguousarray(arr[:, :, [2, 1, 0]])

    if encoding in ("mono8", "8uc1"):
        gray = _rows_from_buffer(buf, height, width, step, 1)[:, :, 0]
        return np.ascontiguousarray(np.stack([gray, gray, gray], axis=-1))

    raise ValueError(f"不支持的图像编码: {msg.encoding}")


def bgr_to_image(arr, msg_template, encoding="bgr8"):
    """BGR uint8 数组 -> sensor_msgs/Image（沿用输入消息的 header）。"""
    out = Image()
    out.header = msg_template.header
    out.height, out.width = arr.shape[0], arr.shape[1]
    out.encoding = encoding
    out.is_bigendian = 0
    out.step = int(arr.shape[1] * arr.shape[2])
    out.data = np.ascontiguousarray(arr).tobytes()
    return out


def gray_to_image(arr, msg_template, encoding="mono8"):
    """灰度 uint8 数组 -> sensor_msgs/Image。"""
    out = Image()
    out.header = msg_template.header
    out.height, out.width = arr.shape[0], arr.shape[1]
    out.encoding = encoding
    out.is_bigendian = 0
    out.step = int(arr.shape[1])
    out.data = np.ascontiguousarray(arr).tobytes()
    return out


def load_config_from_params(rospy_module=rospy):
    """从 ROS 参数服务器读取 LaneFitConfig；缺省用类默认值。

    参数名与 dataclass 字段同名，因此新增字段无需改这里。
    """
    defaults = LaneFitConfig()
    kwargs = {}
    for field_name in defaults.__dataclass_fields__:
        param_name = f"~{field_name}"
        if rospy_module.has_param(param_name):
            kwargs[field_name] = rospy_module.get_param(param_name)
    return LaneFitConfig(**kwargs)


class LaneDetectorNode:
    """把 LaneDetector 接到 ROS 话题上的薄封装。"""

    def __init__(self):
        rospy.init_node("lane_detector", anonymous=False)

        # --- 参数 ---
        self.cfg = load_config_from_params()
        self.publish_debug = rospy.get_param("~publish_debug", True)
        self.debug_every = max(1, int(rospy.get_param("~debug_every", 5)))
        self.departure_threshold_m = float(rospy.get_param("~departure_threshold_m", 1.0))
        self.min_confidence = float(rospy.get_param("~min_confidence", 0.5))
        image_topic = rospy.get_param(
            "~image_topic", "/carla/ego_vehicle/rgb_front/image"
        )

        self.detector = LaneDetector(self.cfg)
        self.bridge = CvBridge() if _HAVE_CV_BRIDGE else None
        self._frame_count = 0

        # --- 发布者 ---
        self.pub_lateral = rospy.Publisher("lane/lateral_offset", Float32, queue_size=1)
        self.pub_heading = rospy.Publisher("lane/heading_error", Float32, queue_size=1)
        self.pub_curvature = rospy.Publisher("lane/curvature", Float32, queue_size=1)
        self.pub_confidence = rospy.Publisher("lane/confidence", Float32, queue_size=1)
        self.pub_departure = rospy.Publisher("lane/departure", Bool, queue_size=1)
        self.pub_mask = (
            rospy.Publisher("lane/mask", Image, queue_size=1)
            if self.publish_debug
            else None
        )

        # --- 订阅者 ---
        self.sub = rospy.Subscriber(
            image_topic, Image, self.on_image, queue_size=1, buff_size=2 ** 24
        )

        rospy.loginfo(
            "[lane_detector] 已启动：订阅 %s，调试掩码发布=%s，cv_bridge=%s",
            image_topic,
            self.publish_debug,
            _HAVE_CV_BRIDGE,
        )

    def on_image(self, msg):
        """单帧回调：解码 -> 检测 -> 发布。"""
        try:
            frame = image_to_bgr(msg, self.bridge)
        except Exception as exc:  # noqa: BLE001
            rospy.logwarn_throttle(5.0, "[lane_detector] 图像解码失败: %s", exc)
            return

        try:
            result = self.detector.detect(frame)
        except Exception as exc:  # noqa: BLE001
            rospy.logerr_throttle(5.0, "[lane_detector] 检测异常: %s", exc)
            return

        self._frame_count += 1
        self.pub_confidence.publish(Float32(data=float(result.confidence)))

        if result.detected and result.confidence >= self.min_confidence:
            self.pub_lateral.publish(Float32(data=float(result.lateral_offset_m)))
            self.pub_heading.publish(Float32(data=float(result.heading_error_rad)))
            curvature = float(result.curvature_m)
            # 直道曲率半径为正无穷，ROS 消息里用 0 表示「直道」
            self.pub_curvature.publish(Float32(data=0.0 if np.isinf(curvature) else curvature))
            self.pub_departure.publish(
                Bool(data=bool(abs(result.lateral_offset_m) > self.departure_threshold_m))
            )
            rospy.loginfo_throttle(
                2.0,
                "[lane_detector] 横向偏差 %+.3f m  航向偏差 %+.2f deg  曲率 %s",
                result.lateral_offset_m,
                np.degrees(result.heading_error_rad),
                "直道" if np.isinf(curvature) else f"{curvature:.0f} m",
            )
        else:
            # 未检出时不要发布 0，避免下游误以为车辆正好居中
            rospy.logwarn_throttle(
                3.0,
                "[lane_detector] 未稳定检出车道线（置信度 %.2f，左 %d / 右 %d 像素）",
                result.confidence,
                result.left_pixels,
                result.right_pixels,
            )

        if self.pub_mask is not None and self._frame_count % self.debug_every == 0:
            mask = self.detector.warped_mask(frame)
            self.pub_mask.publish(
                gray_to_image((mask.astype(np.uint8) * 255), msg, "mono8")
            )


def main():
    try:
        node = LaneDetectorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
