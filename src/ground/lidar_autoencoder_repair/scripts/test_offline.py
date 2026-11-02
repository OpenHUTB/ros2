#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线端到端自测（无需 ROS 环境）。

用 mock 的 rospy / 消息类型直接实例化 main.RepairNode，
喂入合成 LaserScan，检查 /cmd_vel 控制决策是否正确。

覆盖项：
  1. 权重文件存在且成功加载（非随机初始化）
  2. 开阔场景 → 直行 (linear.x=0.3, z=0)，30/30
  3. 正前障碍 + 左侧开阔 → 减速左转 (x=0.05, z=+1.0)，30/30
  4. 正前障碍 + 右侧开阔 → 减速右转 (z=-1.0)
  5. NaN / Inf / 负值输入 → 不崩溃、输出有限
  6. 束数不足 360 → 跳过该帧并告警

用法：
    python3 test_offline.py     # 需 torch + numpy
"""
import os
import sys
import types

import numpy as np

SCRIPTS = os.path.dirname(os.path.abspath(__file__))

# ---------------- mock ROS ----------------
def _mod(name):
    m = types.ModuleType(name)
    sys.modules[name] = m
    return m

LOGS = []
PUBLISHED = []

class ROSInterruptException(Exception):
    pass

class Twist:
    def __init__(self):
        self.linear = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.angular = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)

rospy = _mod("rospy")
rospy.ROSInterruptException = ROSInterruptException
rospy.Twist = Twist
rospy.init_node = lambda *a, **k: None
rospy.spin = lambda *a, **k: None

def _log(*a, **k):
    LOGS.append(" ".join(str(x) for x in a))

rospy.loginfo = _log
rospy.logwarn = _log
rospy.logwarn_throttle = _log

class Publisher:
    def __init__(self, *a, **k):
        pass
    def publish(self, msg):
        PUBLISHED.append(msg)

rospy.Publisher = Publisher
rospy.Subscriber = lambda *a, **k: None

_mod("geometry_msgs")
_mod("geometry_msgs.msg")
sys.modules["geometry_msgs.msg"].Twist = Twist
_mod("sensor_msgs")
_mod("sensor_msgs.msg")

class LaserScan:
    pass

sys.modules["sensor_msgs.msg"].LaserScan = LaserScan

# ---------------- 被测节点 ----------------
sys.path.insert(0, SCRIPTS)
import main as node_mod  # noqa: E402

RESULTS = []

def check(name, cond):
    RESULTS.append((name, bool(cond)))
    print(("PASS " if cond else "FAIL ") + name)

def make_scan(ranges):
    msg = LaserScan()
    msg.ranges = [float(x) for x in ranges]
    return msg

node = node_mod.RepairNode()

def run_once(ranges):
    PUBLISHED.clear()
    node.scan_callback(make_scan(ranges))
    return PUBLISHED[0] if PUBLISHED else None

# 1) 权重加载
check("权重文件存在于 models/", os.path.isfile(
    os.path.normpath(os.path.join(SCRIPTS, "..", "models", "autoencoder_weights.pt"))))
check("启动日志确认权重已加载", any("已加载 Autoencoder 权重" in s for s in LOGS))
check("网络处于 eval 模式（非随机初始化）", not node.autoencoder.training)

# 2) 开阔场景 → 直行
rng = np.random.default_rng(0)
fwd_ok = 0
N = 30
for _ in range(N):
    t = run_once(rng.uniform(2.8, 3.2, 360))
    ok = (t is not None and abs(t.linear.x - 0.3) < 1e-6
          and abs(t.angular.z) < 1e-6
          and np.isfinite([t.linear.x, t.angular.z]).all())
    fwd_ok += ok
check("开阔场景 30/30 直行 (linear.x=0.3, z=0)", fwd_ok == N)

# 3) 正前障碍 + 左侧开阔 → 减速左转
front_ranges = np.full(360, 3.0)
front_ranges[0:15] = 0.4
front_ranges[345:360] = 0.4
front_ranges[15:60] = 2.5    # 左侧开阔
front_ranges[300:345] = 0.5  # 右侧近
turn_ok = 0
for _ in range(N):
    t = run_once(front_ranges)
    ok = (t is not None and abs(t.linear.x - 0.05) < 1e-6
          and abs(t.angular.z - 1.0) < 1e-6)
    turn_ok += ok
check("正前障碍 30/30 减速左转 (x=0.05, z=+1.0)", turn_ok == N)

# 4) 正前障碍 + 右侧开阔 → 减速右转
mirror = np.full(360, 3.0)
mirror[0:15] = 0.4
mirror[345:360] = 0.4
mirror[15:60] = 0.5    # 左侧近
mirror[300:345] = 2.5  # 右侧开阔
t = run_once(mirror)
check("镜像障碍右转 (z=-1.0)", t is not None and abs(t.angular.z + 1.0) < 1e-6)

# 5) NaN / Inf / 负值注入 → 不崩溃、输出有限
bad = rng.uniform(2.8, 3.2, 360).astype(np.float32)
bad[100] = np.nan
bad[200] = np.inf
bad[5] = -1.0
t = run_once(bad)
check("NaN/Inf/负值输入不崩溃且输出有限",
      t is not None and np.isfinite([t.linear.x, t.angular.z]).all())

# 6) 束数不足 360 → 跳过该帧并告警
before_warn = len(LOGS)
run_once(np.full(100, 3.0))
check("束数不足跳过且打印告警", len(LOGS) > before_warn and not PUBLISHED)

passed = sum(1 for _, ok in RESULTS if ok)
print("\n%d/%d PASS" % (passed, len(RESULTS)))
sys.exit(0 if passed == len(RESULTS) else 1)
