# -*- coding: utf-8 -*-
"""图像与 ROS 消息互转工具。

刻意**不使用 cv_bridge**：cv_bridge 在不同机器上经常出现
OpenCV / NumPy ABI 不匹配的问题，而本项目只收发 bgr8 图像，
用 numpy 直接转换只有几行代码，依赖更少、更不容易出 bug。

支持编码：bgr8 / rgb8 / bgra8 / rgba8 / mono8
"""

import numpy as np
from sensor_msgs.msg import Image

# 每种编码的通道数
_CHANNEL_COUNT = {
    'bgr8': 3,
    'rgb8': 3,
    'bgra8': 4,
    'rgba8': 4,
    'mono8': 1,
    '8uc1': 1,
}


def ros_image_to_bgr(msg):
    """sensor_msgs/Image -> numpy 数组（BGR，HxWx3，uint8）。

    参数:
        msg: sensor_msgs.msg.Image
    返回:
        np.ndarray, shape=(H, W, 3), dtype=uint8
    """
    encoding = (msg.encoding or '').lower().strip()
    if encoding not in _CHANNEL_COUNT:
        raise ValueError(
            '不支持的图像编码 "{}"，本工具仅支持 {}'.format(encoding, sorted(_CHANNEL_COUNT))
        )

    height = int(msg.height)
    width = int(msg.width)
    channels = _CHANNEL_COUNT[encoding]

    raw = np.frombuffer(msg.data, dtype=np.uint8)
    row_bytes = width * channels

    # 有些发布者会在每行末尾补齐字节（step > width*channels），这里做兼容
    if int(msg.step) == row_bytes:
        pixels = raw.reshape(height, width, channels)
    else:
        pixels = raw.reshape(height, int(msg.step))[:, :row_bytes].reshape(height, width, channels)

    if encoding == 'bgr8':
        image = pixels
    elif encoding == 'rgb8':
        image = pixels[:, :, ::-1]
    elif encoding == 'bgra8':
        image = pixels[:, :, :3]
    elif encoding == 'rgba8':
        image = pixels[:, :, [2, 1, 0]]
    else:  # mono8 / 8uc1 -> 复制成三通道
        image = np.repeat(pixels, 3, axis=2)

    return np.ascontiguousarray(image)


def bgr_to_ros_image(image, frame_id, stamp):
    """numpy 数组（BGR 或灰度）-> sensor_msgs/Image（encoding=bgr8）。

    参数:
        image:    np.ndarray，HxWx3(BGR) 或 HxW(灰度)，uint8
        frame_id: 坐标系名称字符串
        stamp:    builtin_interfaces/Time
    """
    array = np.asarray(image)
    if array.dtype != np.uint8:
        array = np.clip(array, 0, 255).astype(np.uint8)
    if array.ndim == 2:  # 灰度图扩展成 BGR
        array = np.repeat(array[:, :, None], 3, axis=2)
    if array.ndim != 3 or array.shape[2] != 3:
        raise ValueError('bgr_to_ros_image 只接受 HxWx3 的 BGR 数组，收到 {}'.format(array.shape))
    array = np.ascontiguousarray(array)

    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = int(array.shape[0])
    msg.width = int(array.shape[1])
    msg.encoding = 'bgr8'
    msg.is_bigendian = False
    msg.step = int(array.shape[1] * 3)
    msg.data = array.tobytes()
    return msg
