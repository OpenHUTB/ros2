#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""相机发布节点：CarlaAir/AirSim -> ROS 图像话题.

发布：
    /camera/image_raw   sensor_msgs/Image  encoding=bgr8   前视彩色图（图像尺寸 H x W x 3）
    /camera/depth       sensor_msgs/Image  encoding=32FC1  平面深度图，单位米
    /camera/seg         sensor_msgs/Image  encoding=bgr8   语义分割伪彩色图

设计要点：
  1. 三路图像合并成**一次** `simGetImages` RPC（见 SimClient.get_images_bundle），
     避免每路一次网络往返；实测单次往返 ~1 ms，20 Hz 下仍有充足余量。
  2. 解码只依赖 numpy：AirSim 返回的 BGRA 平面数组去掉 alpha 后即为 BGR，
     正好对应 ROS 的 `bgr8` 编码，无需 OpenCV，减少虚拟机内的依赖。
  3. 深度图用 `DepthPlanar`（ImageType=1）+ `pixels_as_float=True`，得到的是
     **沿光轴的平面深度**（米），而不是透视深度；两者在针孔模型下的关系为
        Z_planar = Z_perspective * cos(theta)
     其中 theta 为该像素相对光轴的夹角。平面深度可直接用于反投影。
  4. header.frame_id 默认取 `frame/body`（base_link），保证在 RViz 中
     能通过 TF world->base_link 正常显示。

参数（默认值见 config/bridge.yaml）：
    sim/host, sim/airsim_port, sim/vehicle_name
    rate/image_hz
    camera/rgb_name, camera/depth_name, camera/seg_name
    camera/publish_rgb, camera/publish_depth, camera/publish_seg
    frame/body
"""
from __future__ import annotations

import os
import sys

import numpy as np
import rospy
from sensor_msgs.msg import Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sim_client import SimClient, decode_depth, decode_rgb  # noqa: E402

# AirSim ImageType 常量（不依赖 airsim 包，便于无仿真时导入本模块做测试）
IMG_SCENE = 0
IMG_DEPTH_PLANAR = 1
IMG_SEGMENTATION = 5


def make_image_msg(arr: np.ndarray, encoding: str, frame_id: str,
                   stamp, seq: int = 0) -> Image:
    """把 numpy 数组打包成 sensor_msgs/Image.

    约定：行优先（row-major）、每行连续存储，因此
        step     = 每行字节数 = width * channels * itemsize
        data_len = step * height
    这是 ROS 图像消息的通用布局，也是 cv_bridge 能够零拷贝读取的前提。
    """
    if arr.ndim == 2:
        height, width = arr.shape
        channels = 1
    elif arr.ndim == 3:
        height, width, channels = arr.shape
    else:
        raise ValueError("图像数组维度非法: %r" % (arr.shape,))

    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.header.seq = seq
    msg.height = int(height)
    msg.width = int(width)
    msg.encoding = encoding
    msg.is_bigendian = False
    msg.step = int(width * channels * arr.dtype.itemsize)
    msg.data = np.ascontiguousarray(arr).tobytes()
    return msg


class ImagePublisher(object):
    """一次 RPC 取 RGB / 深度 / 分割三路图像并分别发布."""

    def __init__(self):
        host = rospy.get_param("sim/host", "127.0.0.1")
        port = int(rospy.get_param("sim/airsim_port", 41451))
        vehicle = rospy.get_param("sim/vehicle_name", "")
        rate_hz = float(rospy.get_param("rate/image_hz", 20.0))

        self.rgb_cam = rospy.get_param("camera/rgb_name", "front_rgb")
        self.depth_cam = rospy.get_param("camera/depth_name", "front_depth")
        self.seg_cam = rospy.get_param("camera/seg_name", "front_seg")
        self.use_rgb = bool(rospy.get_param("camera/publish_rgb", True))
        self.use_depth = bool(rospy.get_param("camera/publish_depth", True))
        self.use_seg = bool(rospy.get_param("camera/publish_seg", False))

        self.topic_rgb = rospy.get_param("topic/image", "/camera/image_raw")
        self.topic_depth = rospy.get_param("topic/depth", "/camera/depth")
        self.topic_seg = rospy.get_param("topic/seg", "/camera/seg")
        self.frame_id = rospy.get_param("frame/body", "base_link")

        self.sim = SimClient(host=host, port=port, vehicle_name=vehicle)
        self.sim.connect()
        rospy.loginfo("image_pub: 已连接仿真器 %s:%d", host, port)

        self.pub_rgb = rospy.Publisher(self.topic_rgb, Image, queue_size=2)
        self.pub_depth = rospy.Publisher(self.topic_depth, Image, queue_size=2)
        self.pub_seg = rospy.Publisher(self.topic_seg, Image, queue_size=2)
        self.rate = rospy.Rate(rate_hz)

        # 采集清单：(相机名, ImageType, pixels_as_float)，顺序固定
        self.specs = []
        if self.use_rgb:
            self.specs.append((self.rgb_cam, IMG_SCENE, False))
        if self.use_depth:
            self.specs.append((self.depth_cam, IMG_DEPTH_PLANAR, True))
        if self.use_seg:
            self.specs.append((self.seg_cam, IMG_SEGMENTATION, False))

        self.seq = 0
        self.fail_count = 0

    # ------------------------------------------------------------------ 主循环
    def spin(self):
        if not self.specs:
            rospy.logwarn("image_pub: 三路图像都被关闭，节点空转")
        while not rospy.is_shutdown():
            try:
                self.step_once()
                self.fail_count = 0
            except Exception as exc:  # noqa: BLE001 - 仿真断连时不能让节点崩溃
                self.fail_count += 1
                if self.fail_count in (1, 10, 100):
                    rospy.logwarn("image_pub: 取图失败(第 %d 次): %s",
                                  self.fail_count, exc)
            self.rate.sleep()

    def step_once(self):
        """取一帧并发布（抽出为独立方法，便于无 ROS 的本地测试）."""
        stamp = rospy.Time.now()
        bundle = self.sim.get_images_bundle(self.specs) if self.specs else []
        for (cam, image_type, _as_float), (_, resp) in zip(self.specs, bundle):
            if resp is None:
                continue
            if image_type == IMG_DEPTH_PLANAR:
                self.pub_depth.publish(
                    make_image_msg(decode_depth(resp), "32FC1", self.frame_id,
                                   stamp, self.seq))
            elif image_type == IMG_SEGMENTATION:
                self.pub_seg.publish(
                    make_image_msg(decode_rgb(resp), "bgr8", self.frame_id,
                                   stamp, self.seq))
            else:
                self.pub_rgb.publish(
                    make_image_msg(decode_rgb(resp), "bgr8", self.frame_id,
                                   stamp, self.seq))
        self.seq += 1


def main():
    rospy.init_node("uav_image_pub", anonymous=False)
    node = ImagePublisher()
    node.spin()


if __name__ == "__main__":
    main()
