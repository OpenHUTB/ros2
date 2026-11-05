#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
main_mujoco_bridge.py —— MuJoCo 仿真桥（ROS Noetic 节点，带可视化窗口）

职责：
  1. 加载 models/ur5e.xml，打开 MuJoCo 可视化窗口，驱动物理仿真
  2. 订阅 /joint_target   (std_msgs/Float64MultiArray, 6 个关节角)   -> 位置伺服目标
  3. 订阅 /grasp_signal   (std_msgs/Bool)                            -> 抓取/释放方块
  4. 订阅 /cube_reset     (std_msgs/Bool)                            -> 方块复位回抓取点
  5. 发布 /joint_states   (sensor_msgs/JointState, 按关节名字读)     -> 供 FSM/RViz 使用

参数：
  ~model_path  MuJoCo XML 路径（默认自动查找）
  ~viewer      是否打开可视化窗口，默认 true；设为 false 则纯后台跑

线程安全约定：
  ROS 回调线程只写"待处理标志"，物理状态的一切修改（含抓取、复位、
  伺服目标）全部在主循环内完成 —— 杜绝 mj_step 与回调的竞争。

注意：mujoco 3.x 必须单独 import mujoco.viewer，否则会报
      module 'mujoco' has no attribute 'viewer'。
"""
import threading

import rospy
import numpy as np
import mujoco
import mujoco.viewer          # ← 3.x 必须显式导入子模块
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Bool

from pickplace_core import PickPlaceCore, find_model_path


class MujocoArmBridge:
    CTRL_HZ = 50.0          # 控制频率（与 core 的 CTRL_HZ 一致）
    N_SUBSTEPS = 10         # 每个控制周期内 mj_step 次数（10*2ms = 20ms/周期）

    def __init__(self):
        model_path = rospy.get_param("~model_path", find_model_path())
        rospy.loginfo(f"Loading MuJoCo model: {model_path}")
        self.core = PickPlaceCore(xml_path=model_path, verbose=False)

        self._lock = threading.Lock()
        self._pending_reset = False
        self.viewer = None

        # 发布 /joint_states（仅 6 个机械臂关节，不受 freejoint 顺序影响）
        self.pub_js = rospy.Publisher("/joint_states", JointState, queue_size=10)
        self.sub_target = rospy.Subscriber("/joint_target", Float64MultiArray,
                                           self.cb_target, queue_size=10)
        self.sub_grasp = rospy.Subscriber("/grasp_signal", Bool,
                                          self.cb_grasp, queue_size=10)
        self.sub_reset = rospy.Subscriber("/cube_reset", Bool,
                                          self.cb_reset, queue_size=10)

    # ---------------- 回调（只置标志，不动物理） ----------------
    def cb_target(self, msg):
        arr = np.asarray(msg.data, dtype=float)
        if arr.size != self.core.nj:
            rospy.logwarn(f"joint_target length {arr.size} != {self.core.nj}, ignored")
            return
        with self._lock:
            self.core.set_target(arr)      # set_target 会做限位裁剪 + 夹爪跟随

    def cb_grasp(self, msg):
        # 距离校验在 core.apply_weld_request 内完成（ee_site-方块中心 < 0.10m）
        self.core.grasp(bool(msg.data))

    def cb_reset(self, msg):
        if msg.data:
            with self._lock:
                self._pending_reset = True

    # ---------------- 打开可视化窗口 ----------------
    def open_viewer(self):
        """打开被动式可视化窗口；失败时打印可执行的排查建议并转无界面模式。"""
        if not rospy.get_param("~viewer", True):
            rospy.logwarn("~viewer=false：无界面模式运行（不显示 MuJoCo 窗口）")
            return
        try:
            self.viewer = mujoco.viewer.launch_passive(self.core.model, self.core.data)
            # 初始相机视角：斜上方俯视整台机械臂
            self.viewer.cam.distance = 2.4
            self.viewer.cam.azimuth = 140.0
            self.viewer.cam.elevation = -22.0
            self.viewer.cam.lookat[:] = np.array([0.0, 0.0, 0.45])
            rospy.loginfo("✅ MuJoCo viewer 窗口已打开（关闭窗口即退出节点）")
        except Exception as e:
            self.viewer = None
            rospy.logerr(f"❌ MuJoCo viewer 打开失败: {e}")
            rospy.logwarn("排查顺序：")
            rospy.logwarn("  1) echo $DISPLAY 是否为空（为空说明没有图形会话）")
            rospy.logwarn("  2) sudo apt install -y libglfw3 libgl1-mesa-glx")
            rospy.logwarn("  3) VMware: 虚拟机设置->显示器->勾选“加速3D图形”，重启虚拟机")
            rospy.logwarn("  4) glxinfo | grep 'OpenGL renderer'（llvmpipe=软渲染，容易失败）")
            rospy.logwarn("  -> 仍失败时可用纯后台模式：rosparam set /mujoco_bridge/viewer false")

    # ---------------- 主循环 ----------------
    def run(self):
        self.open_viewer()
        rospy.loginfo("✅ MuJoCo bridge ready, waiting commands")

        rate = rospy.Rate(self.CTRL_HZ)
        while not rospy.is_shutdown():
            # 用户关闭窗口 -> 退出节点
            if self.viewer is not None and not self.viewer.is_running():
                rospy.loginfo("MuJoCo 窗口已关闭，节点退出")
                break

            with self._lock:
                if self._pending_reset:
                    self.core.reset_cube()
                    self._pending_reset = False
                    rospy.loginfo("🔁 cube reset to grasp point")

            # 伺服目标已由 set_target 写入 core.target_q，step 内部应用
            self.core.step(self.N_SUBSTEPS)

            # 发布关节状态（按名字，任意 qpos 顺序都正确）
            js = JointState()
            js.header.stamp = rospy.Time.now()
            js.name = self.core.joint_names
            js.position = self.core.read_joints().tolist()
            js.velocity = self.core.read_joint_vel().tolist()
            self.pub_js.publish(js)

            # 同步画面
            if self.viewer is not None:
                self.viewer.sync()

            rate.sleep()

        if self.viewer is not None:
            self.viewer.close()


if __name__ == "__main__":
    try:
        rospy.init_node("mujoco_bridge", anonymous=False)
        MujocoArmBridge().run()
    except Exception as e:
        import traceback
        rospy.logerr(f"Bridge error: {e}\n{traceback.format_exc()}")

