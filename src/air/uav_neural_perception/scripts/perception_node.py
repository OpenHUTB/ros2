#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
perception_node.py —— 部署节点：前视图像 -> CNN 推理 -> 发布 /uav/cmd_vel

安全约定：
- 权重缺失或加载失败时立即安全停止并明确报错，绝不使用随机初始化模型驱动无人机；
- 随机权重仅用于离线自检，部署侧不开启该退路；
- 模型输出 NaN / inf 等非法值时跳过本帧，不下发速度指令。
"""
import os
import sys
import rospy
import torch
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

# roslaunch 经 devel/lib relay wrapper 启动时，确保同目录源脚本可被优先导入，
# 避免误命中 devel/lib 下不暴露模块函数的 relay 包装文件。
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from dataset import preprocess_image, denormalize_action
from model import build_model
from safety import (
    load_trained_model,
    sanitize_action,
    WeightMissingError,
    WeightLoadError,
)


class _RosLogger:
    """把 rospy 日志方法适配到 safety 层使用的 info/warn/error 接口。"""

    def info(self, msg):
        rospy.loginfo(msg)

    def warn(self, msg):
        rospy.logwarn(msg)

    def error(self, msg):
        rospy.logerr(msg)


class NeuralPerceptionNode:
    def __init__(self):
        rospy.init_node("uav_neural_perception")
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        self.model_path = rospy.get_param(
            "~model_path",
            os.path.join(os.path.dirname(__file__), "models", "best_drone_model.pth"))
        self.max_vx = float(rospy.get_param("~max_vx", 2.0))
        self.max_vy = float(rospy.get_param("~max_vy", 0.5))
        self.max_yaw = float(rospy.get_param("~max_yaw", 0.4))

        # 在线指令增益：模型回归均值导致动作偏保守，可按轴放大模型已判断出的
        # 方向信号；默认 1.0 保持原始行为。放大后仍由 sanitize_action 兜底限幅。
        self.gain_vx = float(rospy.get_param("~gain_vx", 1.0))
        self.gain_vy = float(rospy.get_param("~gain_vy", 1.0))
        self.gain_yaw = float(rospy.get_param("~gain_yaw", 1.0))

        # 部署侧固定 allow_random=False：权重缺失 / 加载失败在此处安全停止，
        # 不创建发布者、不注册订阅者、不进入推理循环，避免随机模型驱动无人机。
        try:
            self.model = load_trained_model(
                self.model_path, self.device, build_model,
                allow_random=False, log=_RosLogger())
        except (WeightMissingError, WeightLoadError) as exc:
            rospy.logfatal("[安全停止] %s", exc)
            rospy.signal_shutdown(
                "模型权重不可用，节点安全停止，以避免随机模型驱动无人机。")
            return

        self.bridge = CvBridge()
        self.cmd_pub = rospy.Publisher("/uav/cmd_vel", Twist, queue_size=1)
        rospy.Subscriber("/camera/image_raw", Image, self.on_image, queue_size=1)
        rospy.loginfo("神经网络感知节点已启动，等待 /camera/image_raw ...")

    def on_image(self, msg):
        img_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        tensor = preprocess_image(img_bgr).unsqueeze(0).to(self.device)

        with torch.no_grad():
            action_norm = self.model(tensor)[0].cpu().numpy()

        # 模型输出为归一化 [-1, 1] 空间，先反归一化回原始速度指令，
        # 再按轴施加在线增益（默认 1.0 不改变行为），最后交由安全层限幅并
        # 拦截 NaN / inf / 长度异常。
        action = denormalize_action(action_norm)
        action = [action[0] * self.gain_vx,
                  action[1] * self.gain_vy,
                  action[2] * self.gain_yaw]
        result = sanitize_action(action, self.max_vx, self.max_vy, self.max_yaw)
        if result is None:
            # 非法输出（NaN / inf / 长度异常）：丢弃本帧，禁止下发速度指令。
            rospy.logerr_throttle(
                1.0, "模型输出非法（NaN/inf/格式异常），已丢弃本帧指令，不下发速度。")
            return

        vx, vy, yaw = result
        twist = Twist()
        twist.linear.x, twist.linear.y, twist.linear.z = vx, vy, 0.0
        twist.angular.z = yaw
        self.cmd_pub.publish(twist)
        rospy.loginfo_throttle(
            1.0, "下发: vx=%.2f vy=%.2f yaw=%.2f", vx, vy, yaw)


if __name__ == "__main__":
    try:
        NeuralPerceptionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
