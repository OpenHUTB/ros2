#!/usr/bin/env python3
"""
相机真实渲染回归测试
====================
验证 /rov/camera/image_raw 发布的是 MuJoCo 离屏渲染的真实场景图像，
而不是固定颜色的占位缓冲区。

运行（需要可用的离屏 GL 上下文，WSL 下必须 osmesa）：
    MUJOCO_GL=osmesa python3 test/test_camera.py
"""

import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import rclpy  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402


def run_test():
    print("=" * 60)
    print("  相机真实渲染回归测试")
    print("=" * 60)

    import mujoco
    from rov_mujoco.mujoco_sim_node import MujocoSimNode

    rclpy.init(args=['--ros-args', '-p', 'enable_camera_render:=true'])
    node = MujocoSimNode()
    try:
        cam = node.camera
        assert cam is not None, "相机模块未初始化！"

        print(f"[1] 渲染后端: MUJOCO_GL={cam.render_backend}  mode={cam.render_mode}")
        assert cam.render_mode == "real", (
            "相机未启用真实渲染！请设置 MUJOCO_GL=osmesa 后重试。"
        )

        # 推进物理并渲染一帧
        for _ in range(50):
            node._sim_step_callback()
        mujoco.mj_forward(node.model, node.data)
        img_a = cam.render()
        print(f"[2] 渲染图像: shape={img_a.shape} dtype={img_a.dtype} "
              f"mean={img_a.mean():.1f} std={img_a.std():.1f}")
        assert img_a.shape == (cam.height, cam.width, 3)
        assert img_a.dtype == np.uint8
        assert img_a.std() > 5.0, "图像近似常数，说明并未渲染真实场景！"

        # 改变 ROV 位姿后图像必须变化
        q = node.data.qpos
        old = q[node.qpos_addr:node.qpos_addr + 7].copy()
        q[node.qpos_addr + 3] = math.cos(0.35)
        q[node.qpos_addr + 6] = math.sin(0.35)   # 绕 Z 偏航 0.7 rad
        mujoco.mj_forward(node.model, node.data)
        img_b = cam.render()
        q[node.qpos_addr:node.qpos_addr + 7] = old
        mujoco.mj_forward(node.model, node.data)

        diff = float(np.mean(np.abs(img_a.astype(np.int16) - img_b.astype(np.int16))))
        print(f"[3] 位姿变化前后图像平均绝对差: {diff:.2f}")
        assert diff > 1.0, "ROV 位姿改变后画面未变化，渲染未真正跟随仿真状态！"

        # ROS 图像消息内容检查
        received = []
        node.create_subscription(Image, '/rov/camera/image_raw',
                                 lambda msg: received.append(msg), 10)
        time.sleep(0.2)
        for _ in range(10):
            node._publish_callback()
            rclpy.spin_once(node, timeout_sec=0.05)

        assert received, "未收到 /rov/camera/image_raw 消息！"
        msg = received[-1]
        data = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        print(f"[4] 收到 Image 消息: {msg.width}x{msg.height} encoding={msg.encoding} "
              f"step={msg.step} std={data.std():.1f}")
        assert msg.encoding == "rgb8"
        assert msg.step == msg.width * 3
        assert len(msg.data) == msg.height * msg.width * 3
        assert data.std() > 5.0, "话题中的图像是常数图！"

        print("\n✅ 相机真实渲染测试全部通过！")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    run_test()
