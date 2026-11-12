#!/usr/bin/env python3
"""RVNN sonar perception node: neural denoising of the 64-beam scan.

Subscribes ``/rov/scan`` and publishes ``/rov/scan_filtered`` using the trained
``models/perception_mlp.npz`` network (input: 64 normalised ranges, output:
denoised ranges).  If the model or the ``rov_numpy_nn_slam.nn`` package is missing the
node forwards the raw scan unchanged and reports ``MODEL_MISSING`` on
``/rov/status`` so the bringup keeps working.
"""

from __future__ import annotations

import json
import os
import sys
import time


def _repo_root() -> str:
    env = os.environ.get("RVNN_ROOT", "")
    if env:
        cand = os.path.realpath(env)
        if os.path.isfile(os.path.join(cand, "rov_numpy_nn_slam", "runtime.py")):
            return cand
    here = os.path.dirname(os.path.realpath(__file__))
    for cand in (os.path.dirname(here), here):
        if os.path.isfile(os.path.join(cand, "rov_numpy_nn_slam", "runtime.py")):
            return cand
    for entry in os.environ.get("ROS_PACKAGE_PATH", "").split(os.pathsep):
        if not entry:
            continue
        pkg = os.path.join(entry, "rov_numpy_nn_slam")
        if os.path.isfile(os.path.join(pkg, "package.xml")):
            return os.path.dirname(pkg)
    return os.path.dirname(here)


ROOT = _repo_root()
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from rov_numpy_nn_slam.runtime import ensure_venv

ensure_venv(__file__)

import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from rov_numpy_nn_slam import config as C
from rov_numpy_nn_slam.datasets import scan_features

try:  # independent module owned by nn-eng
    from rov_numpy_nn_slam.nn.mlp import MLP
    NN_AVAILABLE = True
    NN_ERROR = ""
except Exception as _exc:  # pragma: no cover - concurrent development
    MLP = None
    NN_AVAILABLE = False
    NN_ERROR = str(_exc)


def _meters_from_prediction(pred: np.ndarray, ranges_in: np.ndarray) -> np.ndarray:
    """Convert a network prediction into sonar ranges in metres.

    The training target is the normalised clean scan (``scan_features``);
    a prediction that already lies in metres is also accepted.
    """
    p = np.asarray(pred, dtype=np.float64).ravel()
    n = min(p.size, ranges_in.size)
    p = p[:n]
    if p.size == 0:
        return np.full(ranges_in.size, C.SONAR_RANGE_MAX, dtype=np.float64)
    finite = p[np.isfinite(p)]
    if finite.size and finite.min() >= -0.05 and finite.max() <= 1.01:
        meters = p * (C.SONAR_RANGE_MAX - C.SONAR_RANGE_MIN) + C.SONAR_RANGE_MIN
    else:
        meters = p
    meters = np.clip(np.nan_to_num(meters, nan=C.SONAR_RANGE_MAX),
                     C.SONAR_RANGE_MIN, C.SONAR_RANGE_MAX)
    out = np.array(ranges_in, dtype=np.float64, copy=True)
    out[:n] = meters
    # a beam with no return stays "no return" after denoising
    out[ranges_in >= C.SONAR_RANGE_MAX * 0.995] = C.SONAR_RANGE_MAX
    return out


class PerceptionNode:
    """Neural denoiser between ``/rov/scan`` and ``/rov/scan_filtered``."""

    def __init__(self) -> None:
        self.model_path = str(rospy.get_param(
            "~model", os.path.join(C.MODELS_DIR, "perception_mlp.npz")))
        in_topic = str(rospy.get_param("~in_topic", "/rov/scan"))
        out_topic = str(rospy.get_param("~out_topic", "/rov/scan_filtered"))
        self.net = None
        self._infer_ms = 0.0
        self._frames = 0
        self._status = "MODEL_MISSING"

        if NN_AVAILABLE and os.path.isfile(self.model_path):
            try:
                net = MLP.load(self.model_path)
                if int(net.sizes[0]) != C.SONAR_BEAMS:
                    rospy.logwarn("perception_node: model input width %d != "
                                  "%d sonar beams", int(net.sizes[0]),
                                  C.SONAR_BEAMS)
                else:
                    self.net = net
                    self._status = "OK"
                    rospy.loginfo("perception_node: loaded %s (in=%d out=%d)",
                                  self.model_path, int(net.sizes[0]),
                                  int(net.sizes[-1]))
            except Exception as exc:  # pragma: no cover - defensive
                rospy.logwarn("perception_node: cannot load %s: %s",
                              self.model_path, exc)
                self._status = "MODEL_BROKEN"
        else:
            rospy.logwarn("perception_node: model %s unavailable (%s) - "
                          "forwarding the raw scan", self.model_path,
                          NN_ERROR or "file not found")

        self.pub_scan = rospy.Publisher(out_topic, LaserScan, queue_size=2)
        self.pub_status = rospy.Publisher("/rov/status", String, queue_size=5)
        rospy.Subscriber(in_topic, LaserScan, self._on_scan, queue_size=2)
        rospy.on_shutdown(self._on_shutdown)
        rospy.loginfo("perception_node: %s -> %s", in_topic, out_topic)

    # ------------------------------------------------------------- callbacks
    def _on_scan(self, msg: LaserScan) -> None:
        ranges = np.asarray(msg.ranges, dtype=np.float64)
        filtered = ranges
        if self.net is not None:
            t0 = time.time()
            try:
                x = scan_features(ranges)[None, :]
                pred = np.asarray(self.net.predict(x), dtype=np.float64)
                filtered = _meters_from_prediction(pred, ranges)
            except Exception as exc:  # pragma: no cover - defensive
                rospy.logwarn_throttle(2.0, "perception_node: inference failed: %s", exc)
                filtered = ranges
            self._infer_ms = 0.9 * self._infer_ms + 0.1 * 1000.0 * (time.time() - t0)
        self._frames += 1

        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = [float(v) for v in filtered]
        out.intensities = []
        self.pub_scan.publish(out)

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
