"""ROS Image 字节解码；不依赖 ROS，可独立验证步幅与颜色顺序。"""
import cv2
import numpy as np


def decode_image(message):
    """按照每行 step 跳过补齐字节，统一转换为 OpenCV BGR。"""
    channels = {"bgr8": 3, "rgb8": 3, "bgra8": 4, "rgba8": 4, "mono8": 1}
    encoding = message.encoding.lower()
    if encoding not in channels:
        raise ValueError(f"不支持的图像编码: {message.encoding}")
    h, w, step = int(message.height), int(message.width), int(message.step)
    count = channels[encoding]
    if h <= 0 or w <= 0 or step < w * count:
        raise ValueError("Image 尺寸或 step 无效")
    raw = np.frombuffer(bytes(message.data), dtype=np.uint8)
    if raw.size < h * step:
        raise ValueError("Image 数据长度小于 height * step")
    pixels = raw[:h * step].reshape(h, step)[:, :w * count].copy()
    if count == 1:
        return cv2.cvtColor(pixels.reshape(h, w), cv2.COLOR_GRAY2BGR)
    image = pixels.reshape(h, w, count)
    code = {"rgb8": cv2.COLOR_RGB2BGR, "rgba8": cv2.COLOR_RGBA2BGR, "bgra8": cv2.COLOR_BGRA2BGR}.get(encoding)
    return cv2.cvtColor(image, code) if code is not None else image
