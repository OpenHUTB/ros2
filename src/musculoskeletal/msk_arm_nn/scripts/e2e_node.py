#!/usr/bin/env python3
"""MSK-Arm end-to-end node: depth image -> CNN -> muscle activations.

Subscribes ``/msk/depth`` (32x32 ``32FC1``, or the RGB camera via
``~camera_topic``) and publishes ``/msk/muscle_cmd`` with the trained
``models/e2e_cnn.npz`` network (four muscle activations in ``[0, 1]``).
Missing model / missing network package degrade to a zero command with a
status message instead of a crash.
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
from std_msgs.msg import Float32MultiArray, String

from msk_arm_nn import config as C

try:  # independent module owned by nn-eng
    from msk_arm_nn.nn.cnn import CNN
    NN_AVAILABLE = True
    NN_ERROR = ""
except Exception as _exc:  # pragma: no cover - concurrent development
    CNN = None
    NN_AVAILABLE = False
    NN_ERROR = str(_exc)

N_MUSCLES = len(getattr(C, "MUSCLES", (0, 0, 0, 0)))


def _image_to_input(msg: Image, channels: int) -> np.ndarray:
    """sensor_msgs/Image -> float32 NCHW in [0, 1] (same layout as training)."""
    height = int(msg.height)
    width = int(msg.width)
    if "32F" in str(msg.encoding or ""):
        data = np.frombuffer(msg.data, dtype=np.float32)
        if data.size < height * width:
            return None
        image = np.clip(data[: height * width].reshape(height, width), 0.0, 1.0)
        if channels <= 1:
            return image[None, None, ...]
        return np.repeat(image[None, None, ...], channels, axis=1)
    data = np.frombuffer(msg.data, dtype=np.uint8)
    if data.size < height * width * 3:
        return None
    image = data[: height * width * 3].reshape(height, width, 3)
    arr = image.astype(np.float32) / 255.0
    arr = np.transpose(arr, (2, 0, 1))[None, ...]
    if channels == 1:
        grey = arr.mean(axis=1, keepdims=True)
        return grey
    if channels == 3:
        return arr
    return np.repeat(arr[:, :1, ...], channels, axis=1)


def _load_cnn(path: str):
    """Load ``models/e2e_cnn.npz``, tolerant to an older stride-1 model."""
    from msk_arm_nn.nn.cnn import CNN
    try:
        return CNN.load(path)
    except Exception:
        with np.load(path, allow_pickle=False) as data:
            in_ch, in_h, in_w = [int(v) for v in data["in_shape"]]
            channels = tuple(int(v) for v in data["conv_channels"])
            fc = int(data["fc"])
            out_dim = int(data["out_dim"])
            flat = int(np.asarray(data["fc_W"]).shape[0])
            payload = {k: np.asarray(data[k]) for k in data.files}
        stride = None
        for candidate in (1, 2, 3, 4, 6, 8):
            try:
                probe = CNN(in_ch, in_h, in_w, conv_channels=channels, fc=fc,
                            out_dim=out_dim, seed=0, stride1=candidate)
            except Exception:
                continue
            if int(probe.flat_dim) == flat:
                stride = candidate
                break
        if stride is None:
            raise RuntimeError("cannot match saved CNN weights to a stride")
        net = CNN(in_ch, in_h, in_w, conv_channels=channels, fc=fc,
                  out_dim=out_dim, seed=0, stride1=stride)
        net.conv1.W[...] = payload["conv1_W"]
        net.conv1.b[...] = payload["conv1_b"]
        net.conv2.W[...] = payload["conv2_W"]
        net.conv2.b[...] = payload["conv2_b"]
        net.fc1.W[...] = payload["fc_W"]
        net.fc1.b[...] = payload["fc_b"]
        net.out.W[...] = payload["out_W"]
        net.out.b[...] = payload["out_b"]
        rospy.logwarn("e2e_node: checkpoint uses conv1_stride=%d, "
                      "reconstructed compatible network", stride)
        return net


class E2ENode:
    """Depth image -> CNN -> ``/msk/muscle_cmd`` closed loop."""

    def __init__(self) -> None:
        self.model_path = str(rospy.get_param(
            "~model", os.path.join(C.MODELS_DIR, "e2e_cnn.npz")))
        self.rate_hz = max(1.0, float(rospy.get_param("~rate", 10.0)))
        self.timeout = max(0.1, float(rospy.get_param("~cmd_timeout", 0.5)))
        camera_topic = str(rospy.get_param("~camera_topic", "/msk/depth"))
        self.net = None
        self._status = "MODEL_MISSING"
        self._cmd = np.zeros(N_MUSCLES)
        self._infer_ms = 0.0
        self._frames = 0
        self._last_frame = -1e9

        if NN_AVAILABLE and os.path.isfile(self.model_path):
            try:
                self.net = _load_cnn(self.model_path)
                self._status = "OK"
                rospy.loginfo("e2e_node: loaded %s (in=%dx%dx%d out=%d)",
                              self.model_path, self.net.in_ch, self.net.in_h,
                              self.net.in_w, self.net.out_dim)
            except Exception as exc:  # pragma: no cover - defensive
                rospy.logwarn("e2e_node: cannot load %s: %s",
                              self.model_path, exc)
                self._status = "MODEL_BROKEN"
        else:
            rospy.logwarn("e2e_node: model %s unavailable (%s) - publishing "
                          "zero activations", self.model_path,
                          NN_ERROR or "file not found")

        self.pub_cmd = rospy.Publisher("/msk/muscle_cmd",
                                       Float32MultiArray, queue_size=5)
        self.pub_status = rospy.Publisher("/msk/status", String, queue_size=5)
        rospy.Subscriber(camera_topic, Image, self._on_image, queue_size=2)
        rospy.on_shutdown(self._on_shutdown)
        rospy.loginfo("e2e_node: %s -> /msk/muscle_cmd", camera_topic)

    # ------------------------------------------------------------- callbacks
    def _on_image(self, msg: Image) -> None:
        self._last_frame = time.time()
        if self.net is None:
            return
        t0 = time.time()
        try:
            x = _image_to_input(msg, int(getattr(self.net, "in_ch", 1)))
            if x is None:
                return
            pred = np.asarray(self.net.predict(x), dtype=np.float64).ravel()
            if pred.size < N_MUSCLES:
                return
            out = np.nan_to_num(pred[:N_MUSCLES], nan=0.0)
            if float(np.min(out)) < -0.05:
                out = (out + 1.0) * 0.5
            self._cmd = np.clip(out, 0.0, 1.0)
            self._frames += 1
        except Exception as exc:  # pragma: no cover - defensive
            rospy.logwarn_throttle(2.0, "e2e_node: inference failed: %s", exc)
        self._infer_ms = 0.9 * self._infer_ms + 0.1 * 1000.0 * (time.time() - t0)

    # ------------------------------------------------------------ publishing
    def _publish_cmd(self) -> None:
        if time.time() - self._last_frame > self.timeout:
            cmd = np.zeros(N_MUSCLES)
        else:
            cmd = self._cmd
        msg = Float32MultiArray()
        msg.data = [float(v) for v in cmd]
        self.pub_cmd.publish(msg)

    def _publish_status(self) -> None:
        payload = {
            "node": "e2e",
            "status": self._status,
            "model": os.path.basename(self.model_path),
            "frames": int(self._frames),
            "infer_ms": round(float(self._infer_ms), 2),
            "cmd": [round(float(v), 3) for v in self._cmd],
        }
        self.pub_status.publish(String(data=json.dumps(payload,
                                                       separators=(",", ":"))))

    # -------------------------------------------------------------------- run
    def spin(self) -> None:
        rate = rospy.Rate(self.rate_hz)
        last_status = 0.0
        while not rospy.is_shutdown():
            self._publish_cmd()
            now = time.time()
            if now - last_status >= 1.0:
                last_status = now
                self._publish_status()
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break

    def _on_shutdown(self) -> None:
        try:
            zeros = Float32MultiArray()
            zeros.data = [0.0] * N_MUSCLES
            self.pub_cmd.publish(zeros)
            self._publish_status()
        except Exception:
            pass


def main() -> int:
    rospy.init_node("e2e_node", anonymous=False)
    node = E2ENode()
    try:
        node.spin()
    finally:
        node._on_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
