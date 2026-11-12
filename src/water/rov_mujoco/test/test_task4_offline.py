#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
水下机器人端到端视觉控制纯网络离线单元测试 (无需 ROS 2 / rclpy / MuJoCo 仿真器)
=============================================================================
供 CI 及本地纯 Python / CPU 环境下快速验证：
  1. 三种视觉策略网络参数量精确断言 (Nature-CNN / ResNet-18 / Spatial-Softmax)
  2. 前向推理输入/输出张量尺寸与 Tanh 动作空间 [-1, 1] 约束检查
  3. Spatial-Softmax 空间软注意力关键点提取验证 (1, 32, 2)
  4. 专家先验权重初始化与零权重回退健壮性检查 (VisuomotorController)
"""

import os
import sys
import unittest
import numpy as np
import torch

# 确保定位到 rov_mujoco 源码路径
THIS_DIR = os.path.dirname(os.path.abspath(__file__))
PKG_ROOT = os.path.dirname(THIS_DIR)
ROV_MODULE_DIR = os.path.join(PKG_ROOT, "rov_mujoco")
for p in [PKG_ROOT, ROV_MODULE_DIR]:
    if p not in sys.path:
        sys.path.insert(0, p)

try:
    from end_to_end_networks import (
        NatureCNNVisuomotorPolicy,
        ResNetVisuomotorPolicy,
        SpatialSoftmaxVisuomotorPolicy,
        VisuomotorController
    )
except ImportError:
    from rov_mujoco.end_to_end_networks import (
        NatureCNNVisuomotorPolicy,
        ResNetVisuomotorPolicy,
        SpatialSoftmaxVisuomotorPolicy,
        VisuomotorController
    )


class TestVisuomotorNetworksOffline(unittest.TestCase):
    """纯 PyTorch 离线单元测试用例集"""

    @classmethod
    def setUpClass(cls):
        cls.device = torch.device("cpu")
        cls.dummy_input = torch.zeros(1, 3, 84, 84, device=cls.device)
        cls.batch_input = torch.zeros(4, 3, 84, 84, device=cls.device)

    def test_nature_cnn_topology_and_params(self):
        """验证 Nature-CNN 参数量 (1,748,260) 与前向输出维度 (1, 4)"""
        model = NatureCNNVisuomotorPolicy(action_dim=4).to(self.device)
        model.eval()

        params = sum(p.numel() for p in model.parameters())
        self.assertEqual(params, 1748260, f"Nature-CNN 参数量应为 1,748,260，实际为 {params}")

        with torch.no_grad():
            out = model(self.dummy_input)
            out_batch = model(self.batch_input)

        self.assertEqual(tuple(out.shape), (1, 4))
        self.assertEqual(tuple(out_batch.shape), (4, 4))
        self.assertTrue(torch.all(out >= -1.0) and torch.all(out <= 1.0))

    def test_resnet18_topology_and_params(self):
        """验证 ResNet-18 参数量 (713,540) 与前向输出维度 (1, 4)"""
        model = ResNetVisuomotorPolicy(action_dim=4).to(self.device)
        model.eval()

        params = sum(p.numel() for p in model.parameters())
        self.assertEqual(params, 713540, f"ResNet-18 参数量应为 713,540，实际为 {params}")

        with torch.no_grad():
            out = model(self.dummy_input)
            out_batch = model(self.batch_input)

        self.assertEqual(tuple(out.shape), (1, 4))
        self.assertEqual(tuple(out_batch.shape), (4, 4))
        self.assertTrue(torch.all(out >= -1.0) and torch.all(out <= 1.0))

    def test_spatial_softmax_topology_and_keypoints(self):
        """验证 Spatial-Softmax 参数量 (56,452) 与 32 组空间特征关键点提取"""
        model = SpatialSoftmaxVisuomotorPolicy(action_dim=4).to(self.device)
        model.eval()

        params = sum(p.numel() for p in model.parameters())
        self.assertEqual(params, 56452, f"Spatial-Softmax 参数量应为 56,452，实际为 {params}")

        with torch.no_grad():
            out = model(self.dummy_input)
            kps = model.extract_keypoints(self.dummy_input)

        self.assertEqual(tuple(out.shape), (1, 4))
        self.assertEqual(tuple(kps.shape), (1, 32, 2))
        self.assertTrue(torch.all(out >= -1.0) and torch.all(out <= 1.0))

    def test_visuomotor_controller_fallback_and_prediction(self):
        """验证在缺失预训练权重时，控制器能够平稳调用专家先验并不崩溃"""
        ctrl = VisuomotorController(
            architecture="spatial_softmax",
            algorithm="bc",
            weights_path="non_existent_weights.pt",
            nominal_speed=0.45,
            device="cpu"
        )

        dummy_img = np.zeros((84, 84, 3), dtype=np.uint8)
        # 模拟中间有深色钢管
        dummy_img[:, 38:46, 0] = 50
        dummy_img[:, 38:46, 1] = 60
        dummy_img[:, 38:46, 2] = 70

        cmd = ctrl.predict_action(dummy_img)

        self.assertIn("vx", cmd)
        self.assertIn("vy", cmd)
        self.assertIn("vz", cmd)
        self.assertIn("wz", cmd)
        self.assertIn("raw_action", cmd)
        self.assertEqual(cmd["architecture"], "spatial_softmax")

        # 具备向前的推进速度
        self.assertGreater(cmd["vx"], 0.1)
        self.assertEqual(len(cmd["raw_action"]), 4)


if __name__ == "__main__":
    unittest.main()
