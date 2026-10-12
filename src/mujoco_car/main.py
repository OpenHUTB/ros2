import mujoco
import mujoco.viewer
import time

xml = """
<mujoco>
  <worldbody>
    <light pos="0 0 3"/>
    <geom type="plane" size="5 5 0.1" rgba="0.8 0.8 0.8 1"/>
    <body name="car" pos="0 0 0.2">
      <joint type="free"/>
      <geom type="box" size="0.3 0.2 0.1" rgba="1 0 0 1"/>
      <geom type="sphere" size="0.1" pos="0.3 0.2 0" rgba="0 0 0 1"/>
      <geom type="sphere" size="0.1" pos="0.3 -0.2 0" rgba="0 0 0 1"/>
    </body>
  </worldbody>
</mujoco>
"""

model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)

with mujoco.viewer.launch_passive(model, data) as viewer:
    for i in range(500):
        data.qvel[0] = 1.0
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.01)

print("仿真结束")
input("按回车键退出...")  
