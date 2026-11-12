"""图像预处理纯逻辑：sensor_msgs/Image -> tensor，不依赖 cv2/cv_bridge。"""
import numpy as np

IMG_H, IMG_W = 120, 160
MAX_VX, MAX_VY, MAX_YAW = 2.0, 0.5, 0.4
ACTION_SCALE = np.array([MAX_VX, MAX_VY, MAX_YAW], dtype=np.float32)

_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
_STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)


def normalize_action(action):
    return np.asarray(action, dtype=np.float32) / ACTION_SCALE


def denormalize_action(action_norm):
    return np.asarray(action_norm, dtype=np.float32) * ACTION_SCALE


def resize_nearest(img, out_h, out_w):
    """最近邻 resize（避免依赖 cv2）。img: (H,W,C) uint8/float。"""
    h, w = img.shape[:2]
    row_idx = (np.linspace(0, h - 1, out_h)).astype(np.int32)
    col_idx = (np.linspace(0, w - 1, out_w)).astype(np.int32)
    return img[np.ix_(row_idx, col_idx)]


def decode_image(encoding, data, width, height):
    """sensor_msgs/Image -> (H,W,3) RGB uint8 数组。支持常见 encoding。"""
    import numpy as np
    if encoding in ("rgb8",):
        arr = np.frombuffer(data, dtype=np.uint8).reshape(height, width, 3)
        return arr
    if encoding in ("bgr8",):
        arr = np.frombuffer(data, dtype=np.uint8).reshape(height, width, 3)
        return arr[:, :, ::-1]  # BGR -> RGB
    if encoding in ("rgba8",):
        arr = np.frombuffer(data, dtype=np.uint8).reshape(height, width, 4)
        return arr[:, :, :3]
    if encoding in ("bgra8",):
        arr = np.frombuffer(data, dtype=np.uint8).reshape(height, width, 4)
        return arr[:, :, 2::-1]  # BGR -> RGB, drop A
    if encoding in ("mono8",):
        arr = np.frombuffer(data, dtype=np.uint8).reshape(height, width)
        return np.stack([arr, arr, arr], axis=-1)
    raise ValueError(f"unsupported image encoding: {encoding}")


def preprocess_image(img_rgb):
    """(H,W,3) RGB uint8 -> (3,IMG_H,IMG_W) float32，ImageNet 归一化。"""
    img = resize_nearest(img_rgb, IMG_H, IMG_W).astype(np.float32) / 255.0
    img = (img - _MEAN) / _STD
    return np.transpose(img, (2, 0, 1)).astype(np.float32)
