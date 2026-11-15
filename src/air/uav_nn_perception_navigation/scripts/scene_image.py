"""Decode AirSim uncompressed Scene images for the trained BGR model."""

import numpy as np


def decode_scene_bgr(data, height, width):
    """Return an HxWx3 BGR image from AirSim BGR or BGRA bytes.

    Existing end-to-end weights were trained on BGR channel order, so dropping
    BGRA's alpha preserves the model input instead of silently swapping colors.
    """
    if height <= 0 or width <= 0:
        raise ValueError('Scene image dimensions must be positive')
    raw = np.frombuffer(data, dtype=np.uint8)
    pixels = height * width
    if raw.size == pixels * 3:
        return raw.reshape(height, width, 3)
    if raw.size == pixels * 4:
        return np.ascontiguousarray(raw.reshape(height, width, 4)[:, :, :3])
    raise ValueError('Scene image must contain 3 or 4 channels per pixel')
