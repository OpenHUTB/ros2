#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
hutb_lidar_nn_avoid: 基于激光雷达的神经网络感知与避障运动控制
==================================================
算法: 多层感知机 (MLP), 输入为 2D 激光雷达距离观测, 输出为
      (线速度 v, 角速度 omega) 控制指令。

感知链路: LDS-01 激光雷达 (/scan)
          -> 角度降采样 + 归一化 (观测向量 x ∈ R^60)
          -> MLP 前向推理 (y = W3·ReLU(W2·ReLU(W1·x+b1)+b2)+b3)
          -> 运动控制指令 (/cmd_vel)

训练方式: 行为克隆 (Behavioral Cloning)
  1) 用"安全专家"规则在仿真中生成 (观测, 指令) 数据集
  2) 最小化 MSE 损失, 反向传播更新网络参数
  3) 训练完成的网络替代专家规则, 实现神经网络感知+控制

本脚本支持两种模式:
  rosrun hutb_lidar_nn_avoid main.py train   # 训练并保存权重
  rosrun hutb_lidar_nn_avoid main.py         # 加载权重实时避障
"""
import json
import math
import sys
import os

import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

N_BINS = 60          # 激光角度分箱数（降采样维数）
FRONT_INDEX = 0      # 前方障碍参考角

# ------------------------------------------------------------
# MLP 定义: 纯 NumPy 实现（便于展示计算原理公式）
#   网络结构: 60 -> 64 (ReLU) -> 32 (ReLU) -> 2 (线性)
# ------------------------------------------------------------
class MLP:
    def __init__(self, dims=(N_BINS, 64, 32, 2), seed=42):
        rng = np.random.RandomState(seed)
        self.W, self.b = [], []
        for i in range(len(dims) - 1):
            # He 初始化: W ~ N(0, sqrt(2/n_in))
            self.W.append(rng.randn(dims[i], dims[i + 1]) * math.sqrt(2.0 / dims[i]))
            self.b.append(np.zeros(dims[i + 1]))

    def forward(self, x):
        """前向传播: y = W3·ReLU(W2·ReLU(W1·x+b1)+b2)+b3"""
        a = x
        for i in range(len(self.W) - 1):
            a = np.maximum(0, a @ self.W[i] + self.b[i])   # ReLU 隐藏层
        return a @ self.W[-1] + self.b[-1]                  # 线性输出层

    def predict(self, x):
        return self.forward(x.reshape(1, -1)).ravel()

    def train_batch(self, X, Y, lr=1e-3, epochs=200, batch=64):
        """行为克隆训练: 反向传播 + SGD
        L = (1/2N)Σ||y - ŷ||²   (MSE)
        δL/δW[l] = (a[l-1])ᵀ · δ[l],  δL/δb[l] = δ[l]
        δ 由输出层向输入层逐层回传"""
        n = X.shape[0]
        for ep in range(epochs):
            idx = np.random.permutation(n)
            for s in range(0, n, batch):
                bi = idx[s:s + batch]
                xb, yb = X[bi], Y[bi]
                # 前向缓存
                acts, zs = [xb], []
                a = xb
                for i in range(len(self.W)):
                    z = a @ self.W[i] + self.b[i]
                    zs.append(z)
                    a = np.maximum(0, z) if i < len(self.W) - 1 else z
                    acts.append(a)
                # 反向传播
                delta = (acts[-1] - yb) / batch            # 输出层梯度
                for l in range(len(self.W) - 1, -1, -1):
                    self.W[l] -= lr * acts[l].T @ delta
                    self.b[l] -= lr * delta.sum(axis=0)
                    if l > 0:
                        delta = (delta @ self.W[l].T) * (zs[l - 1] > 0)
            if ep % 50 == 0:
                yhat = self.forward(X)
                loss = 0.5 * np.mean((yhat - Y) ** 2)
                rospy.loginfo("epoch %d loss=%.6f" % (ep, loss))

    def save(self, path):
        np.savez(path, *self.W, *self.b)

    @staticmethod
    def load(path):
        d = np.load(path)
        n_hid = (len(d.files) - 2) // 2
        m = MLP(dims=(N_BINS, 64, 32, 2))
        m.W = [d['arr_%d' % i] for i in range(n_hid + 1)]
        m.b = [d['arr_%d' % (n_hid + 1 + i)] for i in range(n_hid + 1)]
        return m


# ------------------------------------------------------------
# 观测预处理: 激光 -> 60 维特征向量
# ------------------------------------------------------------
def scan_to_features(msg):
    ranges = np.array(msg.ranges, dtype=np.float32)
    ranges = np.clip(ranges, msg.range_min, msg.range_max)
    # 均匀降采样: 将完整扫描按角度均分为 N_BINS 份, 取每份最小值(最保守)
    n = len(ranges)
    bins = n // N_BINS
    feats = ranges[:bins * N_BINS].reshape(N_BINS, bins).min(axis=1)
    feats = feats / msg.range_max                 # 归一化到 [0,1]
    return feats


# ------------------------------------------------------------
# 专家规则(训练数据生成器): 距离越近 -> 转向越大
# ------------------------------------------------------------
def expert_rule(feats):
    """安全专家: 前方区域平均距离 < 阈值则朝开阔侧转向, 否则直行"""
    front = feats[:12].mean()                      # 前方 60° 区域
    left = feats[12:24].mean()                     # 左前方
    right = feats[-24:-12].mean()                  # 右前方
    v_max, w_max = 0.22, 1.5
    if front < 0.5:
        v = 0.08
        w = w_max * (0.5 if right > left else -0.5) * (1.0 - front)
    else:
        v = v_max * min(1.0, front)
        w = 0.0
    return np.array([v, w])


# ------------------------------------------------------------
# ROS 节点
# ------------------------------------------------------------
class AvoidanceNode:
    def __init__(self):
        self.model_path = rospy.get_param('~model_path',
                                          os.path.join(os.path.dirname(__file__), '..', 'models', 'lidar_mlp.npz'))
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.model = None
        rospy.Subscriber('/scan', LaserScan, self.cb, queue_size=1)

    def cb(self, msg):
        if self.model is None:
            return
        x = scan_to_features(msg)
        v, w = self.model.predict(x)
        v = max(0.0, min(0.22, v))                # 限幅
        w = max(-1.5, min(1.5, w))
        t = Twist()
        t.linear.x = float(v)
        t.angular.z = float(w)
        self.pub.publish(t)

    def load(self):
        p = os.path.expanduser(self.model_path)
        if not os.path.exists(p):
            rospy.logerr('权重文件不存在: %s' % p)
            return False
        self.model = MLP.load(p)
        rospy.loginfo('加载 MLP 权重: %s' % p)
        return True


def train_mode():
    """训练模式: 用专家规则生成数据集并训练 MLP"""
    rospy.init_node('nn_lidar_train')
    rng = np.random.RandomState(0)
    X, Y = [], []
    for _ in range(4000):
        # 随机生成仿真观测(带噪), 用专家规则打标签
        feats = rng.uniform(0.1, 1.0, N_BINS).astype(np.float32)
        # 模拟障碍物: 随机在前方放置障碍
        if rng.rand() < 0.6:
            k = rng.randint(0, 20)
            feats[k:k + 8] = rng.uniform(0.05, 0.6, min(8, N_BINS - k))
        feats = np.clip(feats, 0.02, 1.0)
        X.append(feats)
        Y.append(expert_rule(feats))
    X = np.array(X)
    Y = np.array(Y)
    rospy.loginfo('生成数据集: %d 样本' % X.shape[0])
    mlp = MLP()
    mlp.train_batch(X, Y, lr=5e-3, epochs=600)
    # 评价: 在独立测试集上计算 MSE 与夹角误差
    Xt, Yt = [], []
    for _ in range(500):
        feats = rng.uniform(0.1, 1.0, N_BINS).astype(np.float32)
        if rng.rand() < 0.6:
            k = rng.randint(0, 20)
            feats[k:k + 8] = rng.uniform(0.05, 0.6, min(8, N_BINS - k))
        Xt.append(np.clip(feats, 0.02, 1.0))
        Yt.append(expert_rule(feats))
    yhat = mlp.forward(np.array(Xt))
    mse = float(np.mean((yhat - np.array(Yt)) ** 2))
    rospy.loginfo('测试 MSE=%.6f (v 分量 %.4f, w 分量 %.4f)' %
                  (mse, float(np.mean((yhat[:, 0] - np.array(Yt)[:, 0]) ** 2)),
                   float(np.mean((yhat[:, 1] - np.array(Yt)[:, 1]) ** 2))))
    out = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                       'models', 'lidar_mlp.npz')
    os.makedirs(os.path.dirname(out), exist_ok=True)
    mlp.save(out)
    rospy.loginfo('模型已保存: %s' % out)


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else 'run'
    if mode == 'train':
        train_mode()
        return
    rospy.init_node('nn_lidar_avoidance')
    node = AvoidanceNode()
    if node.load():
        rospy.loginfo('神经网络感知避障控制已启动 (MLP 60->64->32->2)')
        rospy.spin()


if __name__ == '__main__':
    main()
