# -*- coding: utf-8 -*-
"""纯逻辑单元测试（不需要 ROS、不需要 MuJoCo）。

评审指出 package.xml 声明了 python3-pytest 却没有 test/ 目录。
这里用标准库 unittest，`python3 -m unittest discover -s test` 即可运行。
被测的都是"已知答案"的纯函数：给定构造好的日志，结果可以手算出来。
"""
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from diff_drive_odom_calib import config                       # noqa: E402
from diff_drive_odom_calib.calibration import identify          # noqa: E402
from diff_drive_odom_calib.model import OdometryIntegrator      # noqa: E402
from diff_drive_odom_calib.slip_nn import compensate            # noqa: E402


def make_log(dphi_l, dphi_r, ratio_s=1.0, ratio_th=1.0, n=100):
    """构造日志：真值 = 标称口径 × 给定比例，这样闭式解应当精确等于该比例。"""
    r, b = config.R_NOMINAL, config.B_NOMINAL
    out = []
    for _ in range(n):
        ds_od = r * 0.5 * (dphi_l + dphi_r)
        dth_od = r * (dphi_r - dphi_l) / b
        out.append(dict(dphi_left=dphi_l, dphi_right=dphi_r,
                        ds_true=ds_od * ratio_s, dth_true=dth_od * ratio_th))
    return out


class TestIdentify(unittest.TestCase):

    def test_c_s_recovers_known_ratio(self):
        """直行日志里真值是里程计的 0.9 倍 → c_s 应精确等于 0.9。"""
        prm = identify(make_log(1.0, 1.0, ratio_s=0.9), make_log(0.0, 0.0, ratio_s=1.0, ratio_th=1.0))
        self.assertAlmostEqual(prm["c_s"], 0.9, places=9)

    def test_c_theta_recovers_known_ratio(self):
        """纯旋转日志里真角是里程计的 0.95 倍 → c_theta 应精确等于 0.95。"""
        prm = identify(make_log(1.0, 1.0, ratio_s=1.0, ratio_th=1.0),
                       make_log(-1.0, 1.0, ratio_th=0.95))
        self.assertAlmostEqual(prm["c_theta"], 0.95, places=9)

    def test_k_head_zero_for_symmetric_straight(self):
        """左右轮完全对称的直行不该有系统性航向漂移 → k_head = 0。"""
        prm = identify(make_log(1.0, 1.0, ratio_s=0.9), make_log(0.0, 0.0))
        self.assertAlmostEqual(prm["k_head"], 0.0, places=12)


class TestOdometryIntegrator(unittest.TestCase):

    def test_default_params_follow_config(self):
        """默认参数必须与 config 一致（评审指出过 model.py 里曾写死 0.05/0.30）。"""
        it = OdometryIntegrator()
        self.assertAlmostEqual(it.r, config.R_NOMINAL, places=12)
        self.assertAlmostEqual(it.b, config.B_NOMINAL, places=12)

    def test_straight_motion_distance(self):
        """两轮各转 10 rad、c_s=1 时，位移应等于 r × 10。"""
        it = OdometryIntegrator(c_s=1.0, c_theta=1.0, k_head=0.0)
        ds, dth = it.step(10.0, 10.0)
        self.assertAlmostEqual(ds, config.R_NOMINAL * 10.0, places=9)
        self.assertAlmostEqual(dth, 0.0, places=12)


class TestGating(unittest.TestCase):
    """直行时 dθ≈0，网络不该再去乘转角系数（否则放大数值噪声）。"""

    class _ZeroNet(object):
        def predict(self, x):
            return [[1.2, 1.2]]        # 故意让网络"想"放大 20%

    class _IdScaler(object):
        def transform(self, x):
            return x

    def test_heading_untouched_when_dtheta_tiny(self):
        _, dth = compensate(self._ZeroNet(), self._IdScaler(),
                            1.0, 1.0, 0.0, 0.0, ds_odom=0.01, dth_odom=1e-6)
        self.assertAlmostEqual(dth, 1e-6, places=12)

    def test_heading_clamped_to_narrow_band(self):
        """转角补偿被限制在 ±8%：网络给 1.2，也只能放大到 1.08。"""
        _, dth = compensate(self._ZeroNet(), self._IdScaler(),
                            1.0, 1.0, 0.0, 0.0, ds_odom=0.01, dth_odom=1.0)
        self.assertAlmostEqual(dth, 1.08, places=9)


if __name__ == "__main__":
    unittest.main()
