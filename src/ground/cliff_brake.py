#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import mujoco
import mujoco.viewer
import numpy as np
import time
from std_msgs.msg import Float32, Bool

# ================= 1. MuJoCo 物理模型定义 =================
xml_model = """
<mujoco>
    <option gravity="0 0 -9.81"/>
    <worldbody>
        <light pos="0 0 3"/>
        <geom name="floor" type="box" size="3.75 2.0 0.1" pos="-1.25 0 -0.1" rgba="0.8 0.8 0.8 1"/>
        <geom type="box" size="5 2 0.1" pos="7.5 0 -2.0" rgba="0.2 0.2 0.2 1"/>

        <body name="chassis" pos="0 0 0.05">
            <geom type="box" size="0.2 0.15 0.05" rgba="0.2 0.6 0.8 1"/>
            <joint name="chassis_x" type="slide" axis="1 0 0"/>
            <site name="sensor_site" pos="0.2 0 0" size="0.02" rgba="1 0 0 1"/>
        </body>
    </worldbody>

    <actuator>
        <motor joint="chassis_x" ctrlrange="-50 50"/>
    </actuator>
</mujoco>
"""

model = mujoco.MjModel.from_xml_string(xml_model)
data = mujoco.MjData(model)
chassis_x_id = model.joint("chassis_x").id
sensor_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "sensor_site")

# ================= 2. ROS 节点与控制逻辑 =================
class CliffBrakeNode:
    def __init__(self):
        self.pub_dist = rospy.Publisher('/ultrasonic_distance', Float32, queue_size=10)
        self.pub_brake = rospy.Publisher('/brake_status', Bool, queue_size=10)
        self.is_braking = False
        print("ROS 节点已启动！开始悬崖边缘紧急制动测试...")

    def run(self):
        with mujoco.viewer.launch_passive(model, data) as viewer:
            viewer.cam.distance = 4.0
            viewer.cam.lookat = [1.5, 0.0, 0.0] 
            
            step = 0
            while viewer.is_running() and not rospy.is_shutdown():
                chassis_x = data.qpos[model.jnt_qposadr[chassis_x_id]]
                distance_to_cliff = 2.5 - chassis_x
                self.pub_dist.publish(distance_to_cliff)
                
                if distance_to_cliff < 0.3:
                    if not self.is_braking:
                        print(f"⚠️ 警告！前方悬崖！距离 {distance_to_cliff:.2f} 米，紧急制动！")
                        self.pub_brake.publish(True)
                        self.is_braking = True
                    
                    data.ctrl[0] = -50.0  
                    data.qvel[model.jnt_dofadr[chassis_x_id]] *= 0.1 
                else:
                    self.is_braking = False
                    data.ctrl[0] = 10.0 

                mujoco.mj_step(model, data)
                viewer.sync()
                time.sleep(0.01)
                
                if step % 100 == 0:
                    state = "刹车" if self.is_braking else "前进"
                    print(f"ROS数据回传: 车头距悬崖 {distance_to_cliff:.2f} 米, 状态: {state}")
                step += 1

if __name__ == '__main__':
    try:
        rospy.init_node('cliff_brake_simulation', anonymous=True)
        node = CliffBrakeNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
