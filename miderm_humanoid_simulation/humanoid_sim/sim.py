import mujoco
import numpy as np

xml = """
<mujoco model="human">
  <option timestep="0.01"/>
  <worldbody>
    <geom name="ground" type="plane" size="5 5 0.1" rgba="0.9 0.9 0.9 1"/>
    <body name="torso" pos="0 0 1.2">
      <geom type="box" size="0.2 0.15 0.25" rgba="0.2 0.6 0.8 1"/>
      <joint name="root" type="free"/>
      <body name="head" pos="0 0 0.3">
        <geom type="sphere" size="0.15" rgba="0.9 0.5 0.3 1"/>
      </body>
      <body name="left_arm" pos="-0.25 0 0.1">
        <joint name="j_larm" type="hinge" axis="0 1 0"/>
        <geom type="cylinder" fromto="0 0 0 -0.35 0 0" size="0.06" rgba="0.1 0.7 0.4 1"/>
      </body>
      <body name="right_arm" pos="0.25 0 0.1">
        <joint name="j_rarm" type="hinge" axis="0 1 0"/>
        <geom type="cylinder" fromto="0 0 0 0.35 0 0" size="0.06" rgba="0.1 0.7 0.4 1"/>
      </body>
      <body name="left_leg" pos="-0.12 0 -0.3">
        <joint name="j_lleg" type="hinge" axis="0 1 0"/>
        <geom type="cylinder" fromto="0 0 0 0 0 -0.45" size="0.07" rgba="0.5 0.3 0.7 1"/>
      </body>
      <body name="right_leg" pos="0.12 0 -0.3">
        <joint name="j_rleg" type="hinge" axis="0 1 0"/>
        <geom type="cylinder" fromto="0 0 0 0 0 -0.45" size="0.07" rgba="0.5 0.3 0.7 1"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor joint="j_larm" gear="50"/>
    <motor joint="j_rarm" gear="50"/>
    <motor joint="j_lleg" gear="50"/>
    <motor joint="j_rleg" gear="50"/>
  </actuator>
</mujoco>
"""

model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)
renderer = mujoco.Renderer(model, width=640, height=480)

sim_time = 5.0
frames = []
t = 0

while t < sim_time:
    data.ctrl[0] = np.sin(t * 2) * 1.2
    data.ctrl[1] = -np.sin(t * 2) * 1.2
    data.ctrl[2] = np.sin(t * 2) * 0.8
    data.ctrl[3] = -np.sin(t * 2) * 0.8
    mujoco.mj_step(model, data)

    renderer.update_scene(data)
    pixels = renderer.render()
    frames.append(pixels)
    t += model.opt.timestep

# 保存视频，先安装imageio
import imageio
imageio.mimsave('robot_sim.mp4', frames, fps=50)
print
