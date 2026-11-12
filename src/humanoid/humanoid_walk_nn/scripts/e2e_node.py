#!/usr/bin/env python3
"""HNN humanoid end-to-end node: camera image -> CNN -> /humanoid/cmd.

Subscribes ``/humanoid/camera/image_raw`` (32x32x3) and publishes
``/humanoid/cmd`` with the trained ``models/e2e_cnn.npz`` network.  A lidar
safety filter is applied when ``/humanoid/scan`` and
``humanoid_walk_nn.control`` are available.  Missing model / missing network
package degrade to a zero command with a status message instead of a crash.

Prediction formats understood by the node:
  * 2 values normalised to ``[-1, 1]`` -> walk speed / turn rate,
  * 2 values already in m/s and rad/s,
  * 4 values ``[vx, vy, vz, yaw_rate]`` world-frame drive -> projected on the
    heading.
"""

from __future__ import annotations

import json
import math
import os
import sys
import time


def _repo_root() -> str:
    env = os.environ.get("HNN_ROOT", "")
    if env:
        cand = os.path.realpath(env)
        if os.path.isfile(os.path.join(cand, "humanoid_walk_nn", "runtime.py")):
            return cand
    here = os.path.dirname(os.path.realpath(__file__))
    for cand in (os.path.dirname(here), here):
        if os.path.isfile(os.path.join(cand, "humanoid_walk_nn", "runtime.py")):
            return cand
    for entry in os.environ.get("ROS_PACKAGE_PATH", "").split(os.pathsep):
        if not entry:
            continue
        pkg = os.path.join(entry, "humanoid_walk_nn")
        if os.path.isfile(os.path.join(pkg, "package.xml")):
            return os.path.dirname(pkg)
    return os.path.dirname(here)


ROOT = _repo_root()
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from humanoid_walk_nn.runtime import ensure_venv

ensure_venv(__file__)

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String

from humanoid_walk_nn import config as C

try:  # independent module owned by nn-eng
    from humanoid_walk_nn.nn.cnn import CNN
    NN_AVAILABLE = True
    NN_ERROR = ""
except Exception as _exc:  # pragma: no cover - concurrent development
    CNN = None
    NN_AVAILABLE = False
    NN_ERROR = str(_exc)

try:  # optional safety filter from nav-eng
    from humanoid_walk_nn.control import SafetyFilter
    SAFETY_AVAILABLE = True
except Exception:  # pragma: no cover - concurrent development
    SafetyFilter = None
    SAFETY_AVAILABLE = False

LIDAR_BEAMS = int(getattr(C, "LIDAR_BEAMS", 72))
MAX_SPEED = float(getattr(C, "MAX_SPEED", 1.25))
MAX_BACKWARD = float(getattr(C, "MAX_BACKWARD", 0.5))
MAX_TURN_RATE = float(getattr(C, "MAX_TURN_RATE", 1.5))
BEAM_ANGLES = np.linspace(-math.pi, math.pi, LIDAR_BEAMS, endpoint=False)


def _image_to_input(msg: Image) -> np.ndarray:
    """sensor_msgs/Image -> float32 NCHW in [0, 1] (same layout as training)."""
    height = int(msg.height)
    width = int(msg.width)
    if msg.encoding in ("32FC1", "mono8"):
        data = np.frombuffer(msg.data, dtype=np.float32 if "32F" in msg.encoding
                             else np.uint8)
        if data.size < height * width:
            return None
        image = data[: height * width].reshape(height, width).astype(np.float32)
        if "8" in msg.encoding:
            image /= 255.0
        image = np.clip(image, 0.0, 1.0)
        return image[None, None, ...]
    data = np.frombuffer(msg.data, dtype=np.uint8)
    if data.size < height * width * 3:
        return None
    image = data[: height * width * 3].reshape(height, width, 3)
    arr = image.astype(np.float32) / 255.0
    return np.transpose(arr, (2, 0, 1))[None, ...]


def _command_from_prediction(pred: np.ndarray, yaw: float = 0.0):
    """Map a network prediction to ``(walk_speed, turn_rate)``."""
    out = np.asarray(pred, dtype=np.float64).ravel()
    if out.size < 2:
        return None
    if out.size >= 4:
        vx, vy, wz = float(out[0]), float(out[1]), float(out[-1])
        if np.all(np.abs(out[:4]) <= 1.5):
            vx *= MAX_SPEED
            vy *= MAX_SPEED
            wz *= MAX_TURN_RATE
        speed = vx * math.cos(yaw) + vy * math.sin(yaw)
        turn = wz
    else:
        speed, turn = float(out[0]), float(out[1])
        if np.all(np.abs(out[:2]) <= 1.5):
            speed *= MAX_SPEED
            turn *= MAX_TURN_RATE
    speed = float(np.clip(speed, -MAX_BACKWARD, MAX_SPEED))
    turn = float(np.clip(turn, -MAX_TURN_RATE, MAX_TURN_RATE))
    return speed, turn


def _load_cnn(path: str):
    """Load ``models/e2e_cnn.npz``, tolerant to an older stride-1 model."""
    from humanoid_walk_nn.nn.cnn import CNN
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
    """Camera -> CNN -> ``/humanoid/cmd`` closed loop."""

    def __init__(self) -> None:
        self.model_path = str(rospy.get_param(
            "~model", os.path.join(C.MODELS_DIR, "e2e_cnn.npz")))
        self.rate_hz = max(1.0, float(rospy.get_param("~rate", 10.0)))
        self.safety = bool(rospy.get_param("~safety", True))
        self.timeout = max(0.1, float(rospy.get_param("~cmd_timeout", 0.5)))
        camera_topic = str(rospy.get_param("~camera_topic",
                                           "/humanoid/camera/image_raw"))
        self.net = None
        self._status = "MODEL_MISSING"
        self._speed = 0.0
        self._turn = 0.0
        self._infer_ms = 0.0
        self._frames = 0
        self._scan = None
        self._scan_stamp = -1e9
        self._filter = None
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
                          "zero commands", self.model_path,
                          NN_ERROR or "file not found")

        if self.safety and SAFETY_AVAILABLE:
            try:
                self._filter = SafetyFilter()
            except Exception as exc:  # pragma: no cover - defensive
                rospy.logwarn("e2e_node: safety filter unavailable: %s", exc)
                self._filter = None

        self.pub_cmd = rospy.Publisher("/humanoid/cmd", Twist, queue_size=5)
        self.pub_status = rospy.Publisher("/humanoid/status", String,
                                          queue_size=5)
        rospy.Subscriber(camera_topic, Image, self._on_image, queue_size=2)
        rospy.Subscriber("/humanoid/scan", LaserScan, self._on_scan,
                         queue_size=2)
        rospy.on_shutdown(self._on_shutdown)
        rospy.loginfo("e2e_node: %s -> /humanoid/cmd", camera_topic)

    # ------------------------------------------------------------- callbacks
    def _on_scan(self, msg: LaserScan) -> None:
        self._scan = np.asarray(msg.ranges, dtype=np.float64)
        self._scan_stamp = time.time()

    def _on_image(self, msg: Image) -> None:
        self._last_frame = time.time()
        if self.net is None:
            return
        t0 = time.time()
        try:
            x = _image_to_input(msg)
            if x is None:
                return
            pred = self.net.predict(x)
            out = _command_from_prediction(pred)
            if out is None:
                return
            speed, turn = out
            if (self._filter is not None and self._scan is not None
                    and time.time() - self._scan_stamp < 1.0):
                # map the walk command into the filter's velocity layout
                filtered = np.asarray(self._filter.apply(
                    np.array([speed, 0.0, 0.0, turn]), self._scan,
                    BEAM_ANGLES), dtype=np.float64)
                if filtered.size >= 1:
                    speed = float(np.clip(filtered[0], -MAX_BACKWARD,
                                          MAX_SPEED))
            self._speed = float(speed)
            self._turn = float(turn)
            self._frames += 1
        except Exception as exc:  # pragma: no cover - defensive
            rospy.logwarn_throttle(2.0, "e2e_node: inference failed: %s", exc)
        self._infer_ms = 0.9 * self._infer_ms + 0.1 * 1000.0 * (time.time() - t0)

    # ------------------------------------------------------------ publishing
    def _publish_cmd(self) -> None:
        if time.time() - self._last_frame > self.timeout:
            speed, turn = 0.0, 0.0
        else:
            speed, turn = self._speed, self._turn
        msg = Twist()
        msg.linear.x = float(speed)
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.z = float(turn)
        self.pub_cmd.publish(msg)

    def _publish_status(self) -> None:
        payload = {
            "node": "e2e",
            "status": self._status,
            "model": os.path.basename(self.model_path),
            "frames": int(self._frames),
            "infer_ms": round(float(self._infer_ms), 2),
            "cmd": [round(float(self._speed), 3),
                    round(float(self._turn), 3)],
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
            self.pub_cmd.publish(Twist())
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
