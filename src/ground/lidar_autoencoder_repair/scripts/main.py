#!/usr/bin/env python3
"""
基于 Autoencoder 的激光雷达数据修复与避障节点。

核心链路：
  /scan (360 束) → [可选：模拟传感器损坏加噪] → Autoencoder 修复 → 阈值避障 → /cmd_vel

Autoencoder 权重从 ../models/autoencoder_weights.pt 加载，由 train_autoencoder.py 离线训练生成。
若权重文件不存在，节点会用随机初始化权重并打印警告，此时修复效果等价于随机噪声——
**请务必先运行训练脚本生成权重后再部署。**

与纯阈值避障（turtlebot3_lidar_avoid，master 上 src/ground/turtlebot3_lidar_avoid，由 #245 引入）的差异：
  - 输入端多了一层 1D 卷积 Autoencoder 去噪
  - 控制输入是修复后的数据，而非原始 msg.ranges（控制器为简化二分支控制，非对照模块的三态状态机）
  - 当传感器注入高斯/椒盐/丢点噪声时，系统仍能平稳运行
"""
import os

import numpy as np
import rospy
import torch
import torch.nn as nn
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


# ---------------------------------------------------------------------------
# Autoencoder 架构（与 train_autoencoder.py 完全一致）
# ---------------------------------------------------------------------------
class LidarAutoencoder(nn.Module):
    """1D 卷积自编码器（结构须与 train_autoencoder.py 完全一致）。

    卷积负责局部平滑：相邻几束通常打在同一个物体上、距离接近，
    逐束噪声正是靠邻域平均压掉的，同时卷积能保住物体边界的距离跳变。
    """

    def __init__(self, input_dim: int = 360, latent_dim: int = 64):
        super().__init__()
        self.encoder = nn.Sequential(
            nn.Conv1d(1, 16, kernel_size=5, stride=2, padding=2),
            nn.ReLU(inplace=True),
            nn.Conv1d(16, 32, kernel_size=5, stride=2, padding=2),
            nn.ReLU(inplace=True),
            nn.Conv1d(32, latent_dim, kernel_size=5, stride=2, padding=2),
            nn.ReLU(inplace=True),
        )
        self.decoder = nn.Sequential(
            nn.ConvTranspose1d(latent_dim, 32, kernel_size=4, stride=2, padding=1),
            nn.ReLU(inplace=True),
            nn.ConvTranspose1d(32, 16, kernel_size=4, stride=2, padding=1),
            nn.ReLU(inplace=True),
            nn.ConvTranspose1d(16, 1, kernel_size=4, stride=2, padding=1),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.decoder(self.encoder(x.unsqueeze(1))).squeeze(1)


# ---------------------------------------------------------------------------
# 工具函数
# ---------------------------------------------------------------------------
def sanitize_ranges(ranges, max_val: float = 3.5) -> np.ndarray:
    """把 NaN / Inf / ≤0 的值全部替换成 max_val，确保下游逻辑安全。"""
    r = np.asarray(ranges, dtype=np.float32)
    r = np.where(np.isfinite(r), r, max_val)
    r = np.where(r <= 0.0, max_val, r)
    return np.clip(r, 0.05, max_val)


def corrupt_ranges(ranges: np.ndarray, noise_sigma: float = 0.3) -> np.ndarray:
    """对干净的激光扫描注入高斯噪声 + 随机丢点，模拟传感器物理损坏。"""
    noisy = ranges + np.random.normal(0.0, noise_sigma, ranges.shape).astype(np.float32)
    # 5% 概率随机把一些点丢掉（置为 max，等价于传感器没收到回波）
    drop_mask = np.random.random(ranges.shape) < 0.05
    noisy[drop_mask] = 3.5
    return sanitize_ranges(noisy)


# ---------------------------------------------------------------------------
# ROS 节点
# ---------------------------------------------------------------------------
class RepairNode:
    def __init__(self):
        rospy.init_node("lidar_repair_node")

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        # --- 加载 Autoencoder ---
        self.device = torch.device("cpu")
        self.autoencoder = LidarAutoencoder().to(self.device)

        # 权重路径：scripts/main.py 的 ../models/autoencoder_weights.pt
        pkg_dir = os.path.dirname(os.path.abspath(__file__))
        weights_path = os.path.normpath(os.path.join(pkg_dir, "..", "models", "autoencoder_weights.pt"))

        if os.path.isfile(weights_path):
            try:
                state = torch.load(weights_path, map_location=self.device, weights_only=True)
                self.autoencoder.load_state_dict(state)
                self.autoencoder.eval()
                rospy.loginfo("已加载 Autoencoder 权重: %s", weights_path)
            except Exception as exc:
                rospy.logwarn("加载权重失败 (%s)，使用随机初始化", exc)
                self.autoencoder.eval()
        else:
            rospy.logwarn(
                "权重文件不存在: %s\n"
                "当前用随机初始化 Autoencoder，输出无意义。\n"
                "请先运行 scripts/train_autoencoder.py 生成权重。",
                weights_path,
            )
            self.autoencoder.eval()

        rospy.loginfo("Autoencoder 雷达修复节点已启动（模拟噪声注入 + 去噪 + 阈值避障）")

    # ------------------------------------------------------------------
    def scan_callback(self, msg: LaserScan):
        # 1) 读取原始 ranges，并立即做 NaN/Inf 安全过滤
        if len(msg.ranges) < 360:
            rospy.logwarn_throttle(2.0, "收到的 LaserScan 只有 %d 束，期望 360 束，跳过这一帧",
                                   len(msg.ranges))
            return

        raw_ranges = sanitize_ranges(msg.ranges[:360], max_val=3.5)

        # 2) 模拟传感器损坏：注入高斯噪声 + 随机丢点
        corrupted_ranges = corrupt_ranges(raw_ranges, noise_sigma=0.3)

        # 3) Autoencoder 修复
        with torch.no_grad():
            inp = torch.tensor(corrupted_ranges).unsqueeze(0).to(self.device)
            repaired_tensor = self.autoencoder(inp)
        repaired_ranges = sanitize_ranges(repaired_tensor.squeeze(0).numpy(), max_val=3.5)

        # 4) 基于修复后的数据做阈值避障控制
        #    扇区为本模块自定义（对照模块 turtlebot3_lidar_avoid 为 ±30°/30-90°/270-330°）：
        #      前方 = 0-15° + 345-360°
        #      左侧 = 15-60°
        #      右侧 = 300-345°
        front = min(repaired_ranges[0:15].min(), repaired_ranges[345:360].min())
        left = repaired_ranges[15:60].min()
        right = repaired_ranges[300:345].min()

        twist = Twist()
        if front < 0.8:
            # 前方有障碍，转向开阔的一侧
            twist.linear.x = 0.05
            twist.angular.z = 1.0 if left > right else -1.0
        else:
            twist.linear.x = 0.3
            twist.angular.z = 0.0

        self.pub.publish(twist)


# ---------------------------------------------------------------------------
if __name__ == "__main__":
    try:
        node = RepairNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
