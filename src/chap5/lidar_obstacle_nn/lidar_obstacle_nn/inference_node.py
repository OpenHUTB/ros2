"""ROS 2 inference node.

Subscribes to ``/scan`` (sensor_msgs/LaserScan), runs the trained
``LidarMLP`` and publishes:
    - ``/obstacle_class``     : std_msgs/String  (near / far / free)
    - ``/obstacle_confidence`` : std_msgs/Float32 (max softmax prob)

Run with:

    ros2 run lidar_obstacle_nn inference_node --ros-args -p model_path:=...
or via the provided ``launch/main.launch.py``.
"""

from __future__ import annotations

import os
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Float32

from ament_index_python.packages import get_package_share_directory

try:
    import torch  # noqa: WPS433 - optional at module import time
except ImportError:  # pragma: no cover
    torch = None

from lidar_obstacle_nn.model import LidarMLP  # noqa: E402


CLASS_NAMES = ('near', 'far', 'free')
DEFAULT_INPUT_SIZE = 360
DEFAULT_DANGER_DISTANCE = 1.0   # metres inside which we say "near"
DEFAULT_FAR_DISTANCE = 3.0      # metres beyond which we say "free"


class LidarInferenceNode(Node):
    """ROS 2 node wrapping the trained MLP for online classification."""

    def __init__(self) -> None:
        super().__init__('lidar_obstacle_nn_inference')

        # ---- Parameters ------------------------------------------------
        self.declare_parameter('model_path', '')
        self.declare_parameter('input_size', DEFAULT_INPUT_SIZE)
        self.declare_parameter('danger_distance', DEFAULT_DANGER_DISTANCE)
        self.declare_parameter('far_distance', DEFAULT_FAR_DISTANCE)
        self.declare_parameter('device', 'cpu')
        # Default: do NOT use CUDA. The MLP is tiny (~50K params), CPU
        # inference is sub-millisecond, and avoiding CUDA avoids
        # capability-mismatch issues (e.g. very recent GPUs whose
        # compute capability is not yet supported by the stable wheels).
        self.declare_parameter('use_gpu', False)

        model_path = (
            self.get_parameter('model_path').get_parameter_value().string_value
            or os.path.join(
                get_package_share_directory('lidar_obstacle_nn'),
                'models',
                'lidar_mlp.pt',
            )
        )
        input_size = int(
            self.get_parameter('input_size').get_parameter_value().integer_value
        )
        use_gpu = bool(
            self.get_parameter('use_gpu').get_parameter_value().bool_value
        )
        device_name = self.get_parameter('device').get_parameter_value().string_value

        # ---- Device selection -----------------------------------------
        # We deliberately start on CPU. Only when CUDA is explicitly
        # requested (use_gpu:=true or device:=cuda*) will the first
        # /scan callback probe whether a CUDA forward actually works
        # (PyTorch may report cuda.is_available()==True even when the
        # installed wheel lacks the kernel for the host GPU).
        self._device = torch.device('cpu')
        self._cuda_probe_done = False
        self._cuda_disabled_reason: Optional[str] = None
        self._want_cuda = (
            use_gpu or device_name.startswith('cuda')
        ) and torch is not None

        if self._want_cuda and torch.cuda.is_available():
            self.get_logger().info(
                'CUDA requested and torch.cuda.is_available()==True. '
                'Probing forward compatibility on the first scan...'
            )
        elif self._want_cuda:
            # Requested but not usable (CPU-only torch build or no GPU):
            # skip the probe entirely instead of crashing on it.
            self.get_logger().warn(
                'CUDA requested but torch.cuda.is_available()==False '
                '(CPU-only torch build or no GPU). Running on CPU.'
            )
            self._cuda_probe_done = True
        else:
            self.get_logger().info('Using CPU device.')

        # ---- Model loading --------------------------------------------
        self._model = LidarMLP(input_size=input_size)
        if os.path.isfile(model_path):
            state = torch.load(model_path, map_location=self._device)
            if isinstance(state, dict) and 'state_dict' in state:
                state = state['state_dict']
            self._model.load_state_dict(state, strict=False)
            self.get_logger().info('Loaded model weights: %s' % model_path)
        else:
            self.get_logger().warn(
                'Model file not found (%s). Running with random weights - '
                'train the network first with `train_model`.' % model_path
            )
        self._model.to(self._device).eval()

        # ---- Topics ----------------------------------------------------
        scan_qos = QoSProfile(
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.create_subscription(LaserScan, '/scan', self._scan_cb, scan_qos)
        self._class_pub = self.create_publisher(String, '/obstacle_class', 10)
        self._conf_pub = self.create_publisher(Float32, '/obstacle_confidence', 10)

        self.get_logger().info(
            'LiDAR inference node ready: /scan -> /obstacle_class, '
            '/obstacle_confidence'
        )

    def _maybe_probe_cuda(self, sample_tensor: torch.Tensor) -> torch.Tensor:
        """Move the input to CUDA on the first call if requested and possible.

        The probe only runs when CUDA was explicitly requested
        (``use_gpu:=true`` or ``device:=cuda*``) AND
        ``torch.cuda.is_available()`` is True, so machines with a
        CPU-only torch build never touch ``torch.cuda`` at all (CPU-only
        builds raise ``AssertionError`` on any CUDA call, not
        ``RuntimeError``). On any CUDA failure, log a warning and keep
        running on CPU instead of crashing the node.
        """
        if self._cuda_probe_done:
            return sample_tensor.to(self._device)
        self._cuda_probe_done = True
        if not self._want_cuda or not torch.cuda.is_available():
            return sample_tensor.to(self._device)
        try:
            cuda_device = torch.device('cuda')
            probe = sample_tensor.to(cuda_device)
            # Force a tiny forward to surface kernel-load errors here.
            self._model.to(cuda_device)
            _ = self._model(probe)
            # Commit to CUDA if the probe succeeded.
            self._device = cuda_device
            self.get_logger().info(
                'CUDA probe OK. Switched inference to %s.'
                % torch.cuda.get_device_name(0)
            )
            return sample_tensor.to(self._device)
        except (RuntimeError, AssertionError) as exc:
            # RuntimeError: CUDA wheel present but kernel/driver mismatch.
            # AssertionError: CPU-only torch build ("Torch not compiled
            # with CUDA enabled"). Both mean: stay on CPU.
            self._cuda_disabled_reason = str(exc)
            self.get_logger().warn(
                'CUDA forward probe failed (%s). Falling back to CPU.'
                % exc
            )
            # Move model back to CPU.
            self._model.to(torch.device('cpu'))
            self._device = torch.device('cpu')
            return sample_tensor.to(self._device)

    def _scan_cb(self, msg: LaserScan) -> None:
        # Resample the raw scan to the model's expected input size.
        raw = np.asarray(msg.ranges, dtype=np.float32)
        if raw.size != self._model.input_size:
            idx = np.linspace(0, raw.size - 1, self._model.input_size)
            raw = np.interp(idx, np.arange(raw.size), raw)
        tensor = torch.from_numpy(raw).unsqueeze(0)

        # Probe CUDA on the first scan (only if requested); falls back
        # to CPU on any error.
        tensor = self._maybe_probe_cuda(tensor)

        try:
            probs = self._model.predict_proba(tensor)[0]
            cls_id = int(torch.argmax(probs).item())
            confidence = float(probs[cls_id].item())
        except (RuntimeError, AssertionError) as exc:
            # Late CUDA failure: drop to CPU and re-run forward.
            self.get_logger().warn(
                'Forward failed on %s (%s). Re-running on CPU.'
                % (self._device, exc)
            )
            self._model.to(torch.device('cpu')).eval()
            self._device = torch.device('cpu')
            tensor = tensor.to(self._device)
            probs = self._model.predict_proba(tensor)[0]
            cls_id = int(torch.argmax(probs).item())
            confidence = float(probs[cls_id].item())

        # Safety-net fallback: also re-derive the label from the raw
        # distances so that running with random weights is still useful
        # for wiring tests in Gazebo (does not override a confident NN
        # prediction, only used when confidence is low).
        if confidence < 0.45:
            min_range = float(np.nanmin(np.where(np.isfinite(raw), raw, np.inf)))
            if min_range < self.get_parameter('danger_distance').get_parameter_value().double_value:
                cls_id = 0
            elif min_range >= self.get_parameter('far_distance').get_parameter_value().double_value:
                cls_id = 2
            else:
                cls_id = 1

        class_msg = String()
        class_msg.data = CLASS_NAMES[cls_id]
        self._class_pub.publish(class_msg)

        conf_msg = Float32()
        conf_msg.data = confidence
        self._conf_pub.publish(conf_msg)


def main(args=None) -> None:
    """Entry point for `ros2 run lidar_obstacle_nn inference_node`."""
    rclpy.init(args=args)
    node = LidarInferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
