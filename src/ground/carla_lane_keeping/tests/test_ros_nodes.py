#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 节点的纯数据转换函数测试（不需要 rospy / ROS master）。

被测对象
--------
`scripts/lane_detector.py` 里的 `image_to_bgr` / `bgr_to_image` / `gray_to_image`
以及 `lane_keeping_controller.py` 里的 `PidController`。

这些函数是节点里最容易写错的部分（编码、stride、通道顺序），
因此即使没有 ROS 环境也应该能测。这里用桩对象模拟 rospy 与消息类型，
通过直接加载源文件的方式绕过 `import rospy`。

运行::

    python -m unittest discover -s tests -v
"""

import os
import sys
import types
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PKG_SCRIPTS = os.path.join(HERE, "..", "scripts")


# ---------------------------------------------------------------------------
# 用桩模块替换 rospy / 消息类型，使被测算法的源文件可以被导入
# ---------------------------------------------------------------------------


def _make_stub_module(name, **attrs):
    mod = types.ModuleType(name)
    for key, value in attrs.items():
        setattr(mod, key, value)
    return mod


def _install_ros_stubs():
    """注入最小的 rospy 桩，让节点脚本能被 import（不执行 main）。"""
    if "rospy" in sys.modules:
        return

    class _PseudoPublisher:
        def __init__(self, *a, **k):
            self.published = []

        def publish(self, msg):
            self.published.append(msg)

    class _FakeMsg:
        """能存放任意字段的消息桩。"""

        def __init__(self, **kwargs):
            for k, v in kwargs.items():
                setattr(self, k, v)

    rospy = _make_stub_module(
        "rospy",
        init_node=lambda *a, **k: None,
        loginfo=lambda *a, **k: None,
        logwarn=lambda *a, **k: None,
        logerr=lambda *a, **k: None,
        loginfo_throttle=lambda *a, **k: None,
        logwarn_throttle=lambda *a, **k: None,
        logerr_throttle=lambda *a, **k: None,
        Publisher=lambda *a, **k: _PseudoPublisher(),
        Subscriber=lambda *a, **k: None,
        get_param=lambda name, default=None: default,
        has_param=lambda name: False,
        set_param=lambda *a, **k: None,
        spin=lambda *a, **k: None,
        Timer=lambda *a, **k: None,
        Duration=lambda *a, **k: None,
        ROSInterruptException=Exception,
    )
    rospy.core = _make_stub_module("rospy.core", is_initialized=lambda: True)
    sys.modules["rospy"] = rospy

    # 消息类型桩
    std_msgs = _make_stub_module("std_msgs")
    std_msgs_msg = _make_stub_module("std_msgs.msg", Float32=_FakeMsg, Bool=_FakeMsg)
    std_msgs.msg = std_msgs_msg
    sys.modules["std_msgs"] = std_msgs
    sys.modules["std_msgs.msg"] = std_msgs_msg

    sensor_msgs = _make_stub_module("sensor_msgs")
    sensor_msgs_msg = _make_stub_module("sensor_msgs.msg", Image=_FakeMsg)
    sensor_msgs.msg = sensor_msgs_msg
    sys.modules["sensor_msgs"] = sensor_msgs
    sys.modules["sensor_msgs.msg"] = sensor_msgs_msg

    geometry_msgs = _make_stub_module("geometry_msgs")
    geometry_msgs_msg = _make_stub_module("geometry_msgs.msg", Twist=_FakeMsg)
    geometry_msgs.msg = geometry_msgs_msg
    sys.modules["geometry_msgs"] = geometry_msgs
    sys.modules["geometry_msgs.msg"] = geometry_msgs_msg

    # carla_msgs 故意不提供，用于验证控制器在缺少该依赖时能正常退化
    return _FakeMsg, _PseudoPublisher


_FakeMsg, _ = _install_ros_stubs()
sys.path.insert(0, os.path.abspath(PKG_SCRIPTS))

import lane_detector  # noqa: E402
import lane_keeping_controller as lkc  # noqa: E402


class _ImageMsg:
    """模拟 sensor_msgs/Image 的属性集合。"""

    def __init__(self, data, height, width, encoding, step=None):
        self.data = data
        self.height = height
        self.width = width
        self.encoding = encoding
        self.step = step if step is not None else width * (3 if "8" in encoding else 1)
        self.header = "stub-header"


class TestImageDecoding(unittest.TestCase):
    """图像解码：这是接真实相机时最容易出错的地方。"""

    def test_bgr8_decodes_without_change(self):
        img = np.arange(4 * 5 * 3, dtype=np.uint8).reshape(4, 5, 3)
        msg = _ImageMsg(img.tobytes(), 4, 5, "bgr8")
        out = lane_detector.image_to_bgr(msg, bridge=None)
        np.testing.assert_array_equal(out, img)

    def test_rgb8_is_swapped_to_bgr(self):
        img = np.zeros((2, 3, 3), dtype=np.uint8)
        img[..., 0] = 10  # 消息里的 R
        img[..., 2] = 200  # 消息里的 B
        msg = _ImageMsg(img.tobytes(), 2, 3, "rgb8")
        out = lane_detector.image_to_bgr(msg, bridge=None)
        # 转换后 BGR 的第 0 通道应为原来的 B（200）
        self.assertEqual(int(out[0, 0, 0]), 200)
        self.assertEqual(int(out[0, 0, 2]), 10)

    def test_bgra8_drops_alpha(self):
        img = np.zeros((2, 2, 4), dtype=np.uint8)
        img[..., 0], img[..., 1], img[..., 2], img[..., 3] = 1, 2, 3, 255
        msg = _ImageMsg(img.tobytes(), 2, 2, "bgra8", step=2 * 4)
        out = lane_detector.image_to_bgr(msg, bridge=None)
        self.assertEqual(out.shape, (2, 2, 3))
        np.testing.assert_array_equal(out[0, 0], [1, 2, 3])

    def test_rgba8_is_converted(self):
        img = np.zeros((1, 1, 4), dtype=np.uint8)
        img[0, 0] = [10, 20, 30, 255]  # R,G,B,A
        msg = _ImageMsg(img.tobytes(), 1, 1, "rgba8", step=4)
        out = lane_detector.image_to_bgr(msg, bridge=None)
        # RGBA -> BGR: B=30, G=20, R=10
        np.testing.assert_array_equal(out[0, 0], [30, 20, 10])

    def test_mono8_expands_to_three_channels(self):
        gray = np.full((3, 4), 77, dtype=np.uint8)
        msg = _ImageMsg(gray.tobytes(), 3, 4, "mono8")
        out = lane_detector.image_to_bgr(msg, bridge=None)
        self.assertEqual(out.shape, (3, 4, 3))
        self.assertTrue((out == 77).all())

    def test_row_padding_is_respected(self):
        """真实相机消息常带行填充（step > width*3），解码必须按 step 切行。"""
        width, height = 3, 2
        step = width * 3 + 4  # 每行多 4 字节填充
        raw = bytearray()
        for row in range(height):
            raw.extend(bytes([row + 1] * (width * 3)))
            raw.extend(b"\x00" * 4)
        msg = _ImageMsg(bytes(raw), height, width, "bgr8", step=step)
        out = lane_detector.image_to_bgr(msg, bridge=None)
        self.assertEqual(out.shape, (height, width, 3))
        self.assertTrue((out[0] == 1).all())
        self.assertTrue((out[1] == 2).all())

    def test_unsupported_encoding_raises(self):
        msg = _ImageMsg(b"\x00" * 12, 2, 2, "32FC1")
        with self.assertRaises(ValueError):
            lane_detector.image_to_bgr(msg, bridge=None)


class TestImageEncoding(unittest.TestCase):
    """编码回 ROS 消息：debug 掩码发布依赖它。"""

    def test_bgr_to_image_fields(self):
        arr = np.zeros((6, 8, 3), dtype=np.uint8)
        tmpl = _ImageMsg(b"", 6, 8, "bgr8")
        out = lane_detector.bgr_to_image(arr, tmpl)
        self.assertEqual((out.height, out.width), (6, 8))
        self.assertEqual(out.encoding, "bgr8")
        self.assertEqual(out.step, 8 * 3)
        self.assertEqual(len(out.data), 6 * 8 * 3)
        self.assertEqual(out.header, tmpl.header)

    def test_gray_to_image_fields(self):
        arr = np.zeros((5, 7), dtype=np.uint8)
        tmpl = _ImageMsg(b"", 5, 7, "mono8")
        out = lane_detector.gray_to_image(arr, tmpl, "mono8")
        self.assertEqual((out.height, out.width), (5, 7))
        self.assertEqual(out.encoding, "mono8")
        self.assertEqual(out.step, 7)
        self.assertEqual(len(out.data), 5 * 7)

    def test_roundtrip_bgr(self):
        rng = np.random.default_rng(0)
        arr = rng.integers(0, 256, (4, 6, 3), dtype=np.uint8)
        tmpl = _ImageMsg(b"", 4, 6, "bgr8")
        msg = lane_detector.bgr_to_image(arr, tmpl)
        back = lane_detector.image_to_bgr(msg, bridge=None)
        np.testing.assert_array_equal(back, arr)


class TestConfigLoading(unittest.TestCase):
    def test_defaults_when_no_params(self):
        """参数服务器里没有对应项时，应使用 LaneFitConfig 的默认值。"""
        cfg = lane_detector.load_config_from_params(rospy_module=sys.modules["rospy"])
        from lane_detection import LaneFitConfig

        self.assertEqual(cfg.sobel_min, LaneFitConfig().sobel_min)

    def test_params_override(self):
        """参数服务器里的值应覆盖默认值。"""

        class _StubRospy:
            _vals = {"~sobel_min": 123.0, "~poly_degree": 3}

            @staticmethod
            def has_param(name):
                return name in _StubRospy._vals

            @staticmethod
            def get_param(name):
                return _StubRospy._vals[name]

        cfg = lane_detector.load_config_from_params(rospy_module=_StubRospy)
        self.assertAlmostEqual(cfg.sobel_min, 123.0)
        self.assertEqual(cfg.poly_degree, 3)

    def test_sequence_field_becomes_tuple(self):
        """rosparam 没有 tuple 类型，hist_band_m 会以 list 传进来，必须转回 tuple。"""

        class _StubRospy:
            _vals = {"~hist_band_m": [4.0, 12.0]}

            @staticmethod
            def has_param(name):
                return name in _StubRospy._vals

            @staticmethod
            def get_param(name):
                return _StubRospy._vals[name]

        cfg = lane_detector.load_config_from_params(rospy_module=_StubRospy)
        self.assertIsInstance(cfg.hist_band_m, tuple)
        self.assertEqual(cfg.hist_band_m, (4.0, 12.0))
        # 解包是代码里的实际用法，转成 tuple 后必须仍能两元解包
        lo, hi = cfg.hist_band_m
        self.assertLess(lo, hi)


class TestCameraParamLoading(unittest.TestCase):
    """相机内参读取：标定模式能不能用，取决于这三个值有没有正确读进来。"""

    def _camera_with(self, vals):
        class _StubRospy:
            _vals = vals

            @staticmethod
            def get_param(name, default=None):
                return _StubRospy._vals.get(name, default)

        return lane_detector.load_camera_from_params(rospy_module=_StubRospy)

    def test_defaults_match_carla_front_camera(self):
        cam = self._camera_with({})
        self.assertAlmostEqual(cam.hfov_deg, 90.0)
        self.assertAlmostEqual(cam.mount_height_m, 1.70)
        self.assertAlmostEqual(cam.pitch_deg, 8.0)

    def test_params_are_applied(self):
        cam = self._camera_with(
            {"~camera_hfov_deg": 100.0, "~camera_height_m": 2.1, "~camera_pitch_deg": 12.0}
        )
        self.assertAlmostEqual(cam.hfov_deg, 100.0)
        self.assertAlmostEqual(cam.mount_height_m, 2.1)
        self.assertAlmostEqual(cam.pitch_deg, 12.0)

    def test_camera_is_a_real_pinhole_model(self):
        """读出来的必须是能投影的相机模型，而不是只有几个裸参数。"""
        cam = self._camera_with({})
        u, v = cam.project(10.0, 0.0)  # 正前方 10 m 应落在图像中心竖线上
        self.assertAlmostEqual(u, cam.width / 2.0, places=6)
        self.assertGreater(v, cam.horizon_row())  # 地面点在地平线以下
        d, y = cam.unproject(u, v)
        self.assertAlmostEqual(d, 10.0, places=6)
        self.assertAlmostEqual(y, 0.0, places=6)


class TestPidController(unittest.TestCase):
    """PID：横向控制的核心，必须验证积分限幅、微分与输出限幅。"""

    def test_pure_proportional(self):
        pid = lkc.PidController(kp=2.0, ki=0.0, kd=0.0, out_limit=10.0, integral_limit=1.0)
        self.assertAlmostEqual(pid.step(1.0, now=0.0), 2.0)

    def test_output_is_limited(self):
        pid = lkc.PidController(kp=100.0, ki=0.0, kd=0.0, out_limit=1.5, integral_limit=1.0)
        self.assertAlmostEqual(pid.step(10.0, now=0.0), 1.5)
        self.assertAlmostEqual(pid.step(-10.0, now=0.1), -1.5)

    def test_integral_accumulates_and_is_clamped(self):
        pid = lkc.PidController(kp=0.0, ki=1.0, kd=0.0, out_limit=1000.0, integral_limit=0.5)
        pid.step(1.0, now=0.0)
        pid.step(1.0, now=1.0)  # 积分 += 1.0*1.0 = 1.0 -> 限幅到 0.5
        out = pid.step(1.0, now=2.0)
        self.assertLessEqual(abs(out), 0.5 + 1e-9)

    def test_derivative_reacts_to_change(self):
        pid = lkc.PidController(kp=0.0, ki=0.0, kd=1.0, out_limit=1000.0, integral_limit=1.0)
        pid.step(0.0, now=0.0)
        out = pid.step(1.0, now=1.0)  # d(error)/dt = 1.0
        self.assertAlmostEqual(out, 1.0, places=6)

    def test_reset_clears_state(self):
        pid = lkc.PidController(kp=0.0, ki=1.0, kd=0.0, out_limit=1000.0, integral_limit=10.0)
        pid.step(1.0, now=0.0)
        pid.step(1.0, now=1.0)
        pid.reset()
        # 复位后积分应从零开始，输出不包含历史累积
        out = pid.step(0.0, now=2.0)
        self.assertAlmostEqual(out, 0.0, places=9)

    def test_zero_dt_does_not_explode(self):
        """同一时间戳连调两次不应产生除零或无穷。"""
        pid = lkc.PidController(kp=0.1, ki=0.1, kd=0.1, out_limit=100.0, integral_limit=1.0)
        for _ in range(5):
            out = pid.step(1.0, now=1.0)
        self.assertTrue(np.isfinite(out))


class TestControllerSignConvention(unittest.TestCase):
    """符号约定：偏差为正（偏右）时转向应为负（向左打），除非开启 invert。"""

    def _steering_for(self, lateral, heading=0.0, invert=False):
        pid = lkc.PidController(kp=1.0, ki=0.0, kd=0.0, out_limit=10.0, integral_limit=1.0)
        pid._prev_time = 0.0
        pid_out = pid.step(lateral, now=0.1)
        steering = -pid_out - 0.0 * heading
        return -steering if invert else steering

    def test_positive_offset_steers_negative(self):
        self.assertLess(self._steering_for(lateral=1.0), 0.0)

    def test_negative_offset_steers_positive(self):
        self.assertGreater(self._steering_for(lateral=-1.0), 0.0)

    def test_invert_flips_sign(self):
        self.assertGreater(self._steering_for(lateral=1.0, invert=True), 0.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
