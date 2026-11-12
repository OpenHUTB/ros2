#!/usr/bin/env python3
"""MSK-Arm perception node: neural denoising of the workspace depth image.

Subscribes ``/msk/depth`` (sensor_msgs/Image, 32x32 ``32FC1``) and publishes
``/msk/depth_filtered`` using the trained ``models/perception_mlp.npz`` network.
If the model or the ``msk_arm_nn.nn`` package is missing, the node forwards the
raw depth image unchanged and reports ``MODEL_MISSING`` on ``/msk/status`` so
the rest of the bringup keeps working.
"""

from __future__ import annotations

import json
import os
import sys
import time


def _repo_root() -> str:
    env = os.environ.get("MSK_ROOT", "")
    if env:
        cand = os.path.realpath(env)
        if os.path.isfile(os.path.join(cand, "msk_arm_nn", "runtime.py")):
            return cand
    here = os.path.dirname(os.path.realpath(__file__))
    for cand in (os.path.dirname(here), here):
        if os.path.isfile(os.path.join(cand, "msk_arm_nn", "runtime.py")):
            return cand
    for entry in os.environ.get("ROS_PACKAGE_PATH", "").split(os.pathsep):
        if not entry:
            continue
        pkg = os.path.join(entry, "msk_arm_nn")
        if os.path.isfile(os.path.join(pkg, "package.xml")):
            return os.path.dirname(pkg)
    return os.path.dirname(here)


ROOT = _repo_root()
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from msk_arm_nn.runtime import ensure_venv

ensure_venv(__file__)

import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String

from msk_arm_nn import config as C

try:  # independent module owned by nn-eng
    from msk_arm_nn.nn.mlp import MLP
    NN_AVAILABLE = True
    NN_ERROR = ""
except Exception as _exc:  # pragma: no cover - concurrent development
    MLP = None
    NN_AVAILABLE = False
    NN_ERROR = str(_exc)

CAM_H = int(getattr(C, "CAM_HEIGHT", 32))
CAM_W = int(getattr(C, "CAM_WIDTH", 32))


def _image_to_array(msg: Image) -> np.ndarray:
    """sensor_msgs/Image -> float32 (H, W) depth/proximity map in [0, 1]."""
    height = int(msg.height)
    width = int(msg.width)
    encoding = str(msg.encoding or "")
    if "32F" in encoding:
        data = np.frombuffer(msg.data, dtype=np.float32)
    else:
        data = np.frombuffer(msg.data, dtype=np.uint8).astype(np.float32) / 255.0
    if data.size < height * width:
        raise ValueError("depth image payload too small")
    return np.clip(data[: height * width].reshape(height, width), 0.0, 1.0)


def _array_to_image(array: np.ndarray, header) -> Image:
    array = np.asarray(array, dtype=np.float32)
    msg = Image()
    msg.header = header
    msg.height = int(array.shape[0])
    msg.width = int(array.shape[1])
    msg.encoding = "32FC1"
    msg.is_bigendian = 0
    msg.step = int(array.shape[1] * 4)
    msg.data = np.ascontiguousarray(array, dtype=np.float32).tobytes()
    return msg


class PerceptionNode:
    """Neural denoiser between ``/msk/depth`` and ``/msk/depth_filtered``."""

    def __init__(self) -> None:
        self.model_path = str(rospy.get_param(
            "~model", os.path.join(C.MODELS_DIR, "perception_mlp.npz")))
        in_topic = str(rospy.get_param("~in_topic", "/msk/depth"))
        out_topic = str(rospy.get_param("~out_topic", "/msk/depth_filtered"))
        self.net = None
        self._infer_ms = 0.0
        self._frames = 0
        self._status = "MODEL_MISSING"
        self._input_size = None

        if NN_AVAILABLE and os.path.isfile(self.model_path):
            try:
                net = MLP.load(self.model_path)
                self._input_size = int(net.sizes[0])
                self.net = net
                self._status = "OK"
                rospy.loginfo("perception_node: loaded %s (in=%d out=%d)",
                              self.model_path, self._input_size,
                              int(net.sizes[-1]))
            except Exception as exc:  # pragma: no cover - defensive
                rospy.logwarn("perception_node: cannot load %s: %s",
                              self.model_path, exc)
                self._status = "MODEL_BROKEN"
        else:
            rospy.logwarn("perception_node: model %s unavailable (%s) - "
                          "forwarding the raw depth image", self.model_path,
                          NN_ERROR or "file not found")

        self.pub_depth = rospy.Publisher(out_topic, Image, queue_size=2)
        self.pub_status = rospy.Publisher("/msk/status", String, queue_size=5)
        rospy.Subscriber(in_topic, Image, self._on_depth, queue_size=2)
        rospy.on_shutdown(self._on_shutdown)
        rospy.loginfo("perception_node: %s -> %s", in_topic, out_topic)

    # ------------------------------------------------------------- callbacks
    def _on_depth(self, msg: Image) -> None:
        self._frames += 1
        try:
            image = _image_to_array(msg)
        except Exception as exc:  # pragma: no cover - defensive
            rospy.logwarn_throttle(2.0, "perception_node: bad depth image: %s",
                                   exc)
            return
        filtered = image
        if self.net is not None:
            if self._input_size is not None and image.size != self._input_size:
                if self._status == "OK":
                    self._status = "MODEL_WIDTH_MISMATCH"
                    rospy.logwarn("perception_node: model input %d != depth "
                                  "pixels %d - forwarding raw image",
                                  self._input_size, image.size)
            else:
                t0 = time.time()
                try:
                    pred = np.asarray(self.net.predict(
                        image.reshape(1, -1).astype(np.float64)),
                        dtype=np.float64).ravel()
                    filtered = np.clip(np.nan_to_num(pred.reshape(image.shape)),
                                       0.0, 1.0)
                except Exception as exc:  # pragma: no cover - defensive
                    rospy.logwarn_throttle(2.0,
                                           "perception_node: inference failed: %s",
                                           exc)
                    filtered = image
                self._infer_ms = (0.9 * self._infer_ms
                                  + 0.1 * 1000.0 * (time.time() - t0))
        self.pub_depth.publish(_array_to_image(filtered, msg.header))

    # ------------------------------------------------------------ publishing
    def _publish_status(self) -> None:
        payload = {
            "node": "perception",
            "status": self._status,
            "model": os.path.basename(self.model_path),
            "frames": int(self._frames),
            "infer_ms": round(float(self._infer_ms), 2),
        }
        if self.net is not None:
            payload["input"] = int(self.net.sizes[0])
            payload["output"] = int(self.net.sizes[-1])
        self.pub_status.publish(String(data=json.dumps(payload,
                                                       separators=(",", ":"))))

    # -------------------------------------------------------------------- run
    def spin(self) -> None:
        rate = rospy.Rate(1.0)
        while not rospy.is_shutdown():
            self._publish_status()
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break

    def _on_shutdown(self) -> None:
        try:
            self._publish_status()
        except Exception:
            pass


def main() -> int:
    rospy.init_node("perception_node", anonymous=False)
    node = PerceptionNode()
    try:
        node.spin()
    finally:
        node._on_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
