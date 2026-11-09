"""Only rgb8/bgr8/32FC1: NumPy transport, no compiled cv_bridge/OpenCV ABI."""
import numpy as np


def encode_image(array, encoding):
    array = np.asarray(array)
    if encoding in ('rgb8', 'bgr8'):
        if array.ndim != 3 or array.shape[2] != 3 or array.dtype != np.uint8:
            raise ValueError('Color image must be HxWx3 uint8')
        array = np.ascontiguousarray(array)
    elif encoding == '32FC1':
        if array.ndim != 2:
            raise ValueError('Depth image must be HxW')
        array = np.ascontiguousarray(array, dtype='<f4')
    else:
        raise ValueError('Unsupported encoding: ' + encoding)
    if min(array.shape[:2]) <= 0:
        raise ValueError('Empty image')
    return dict(height=array.shape[0], width=array.shape[1], encoding=encoding,
                is_bigendian=0, step=array.shape[1] * (4 if encoding == '32FC1' else 3),
                data=array.tobytes())


class ImageCodec:
    def to_message(self, array, encoding):
        from sensor_msgs.msg import Image
        message = Image()
        for key, value in encode_image(array, encoding).items():
            setattr(message, key, value)
        return message

    def to_array(self, message, desired_encoding):
        source = message.encoding
        if source not in ('rgb8', 'bgr8', '32FC1'):
            raise ValueError('Unsupported encoding: ' + source)
        h, w, step = int(message.height), int(message.width), int(message.step)
        channels, itemsize = (1, 4) if source == '32FC1' else (3, 1)
        if min(h, w) <= 0 or step < w * channels * itemsize:
            raise ValueError('Invalid image dimensions/step')
        raw = memoryview(message.data)
        if raw.nbytes != h * step:
            raise ValueError('Image data length does not match dimensions/step')
        if source == '32FC1':
            if desired_encoding != '32FC1':
                raise ValueError('Depth conversion requires 32FC1')
            dtype = '>f4' if message.is_bigendian else '<f4'
            return np.ndarray((h, w), dtype=dtype, buffer=raw,
                              strides=(step, 4)).astype(np.float32, copy=True)
        if desired_encoding not in ('rgb8', 'bgr8'):
            raise ValueError('Color conversion requires rgb8 or bgr8')
        array = np.ndarray((h, w, 3), dtype=np.uint8, buffer=raw,
                           strides=(step, 3, 1))
        return (array[:, :, ::-1] if source != desired_encoding else array).copy()
