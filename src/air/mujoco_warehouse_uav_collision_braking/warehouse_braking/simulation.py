"""固定姿态无人机平移动力学与真实射线测距。 / Fixed-attitude UAV dynamics and ray ranging."""

from __future__ import annotations

import numpy as np
import mujoco

DT = 0.02
ANGLES = np.deg2rad(np.linspace(-60, 60, 9))
MAX_RANGE = 8.0
MASS = 0.8
RADIUS = 0.18


def scene_xml(scenario: str, obstacle_x: float = 6.0) -> str:
    """构造仓储通道；横穿物体采用运动学驱动。 / Build an aisle with a kinematic crossing obstacle."""
    if scenario not in ("static", "crossing", "narrow"):
        raise ValueError("场景必须为 static/crossing/narrow。 / Unknown scenario.")
    half_width = 0.85 if scenario == "narrow" else 1.6
    obstacle_y = 2.5 if scenario == "crossing" else 0.0
    return f'''<mujoco model="warehouse_uav_braking">
      <option timestep="{DT}" gravity="0 0 -9.81" integrator="implicitfast"/>
      <visual><global offwidth="800" offheight="600"/></visual>
      <worldbody>
        <light pos="4 0 8" dir="0 0 -1"/>
        <geom name="floor" type="plane" size="15 5 .1" rgba=".18 .21 .25 1"/>
        <geom name="left_rack" type="box" pos="5 {half_width + .2} 1" size="6 .2 1" rgba=".35 .5 .7 1"/>
        <geom name="right_rack" type="box" pos="5 {-half_width - .2} 1" size="6 .2 1" rgba=".35 .5 .7 1"/>
        <geom name="end_wall" type="box" pos="12 0 1" size=".1 2 1"/>
        <body name="obstacle" mocap="true" pos="{obstacle_x} {obstacle_y} 1">
          <geom name="obstacle_box" type="box" size=".3 .35 .7" rgba=".95 .45 .15 1"/>
        </body>
        <body name="uav" pos="0 0 1">
          <joint name="x" type="slide" axis="1 0 0" damping=".08"/>
          <joint name="y" type="slide" axis="0 1 0" damping=".08"/>
          <joint name="z" type="slide" axis="0 0 1" damping=".08"/>
          <geom name="uav_hull" type="sphere" size="{RADIUS}" mass="{MASS}" rgba=".2 .85 .75 1"/>
          <geom type="capsule" fromto="-.3 -.3 0 .3 .3 0" size=".025" mass="0" contype="0" conaffinity="0"/>
          <geom type="capsule" fromto="-.3 .3 0 .3 -.3 0" size=".025" mass="0" contype="0" conaffinity="0"/>
        </body>
      </worldbody>
    </mujoco>'''


class WarehouseSimulation:
    """以力控制平移，姿态固定；单位为米、秒。 / Force-controlled translation with fixed attitude, SI units."""

    def __init__(self, scenario="static", obstacle_x=6.0, speed=1.0, crossing_speed=0.8):
        self.scenario = scenario
        self.crossing_speed = float(crossing_speed)
        self.obstacle_x = float(obstacle_x)
        self.model = mujoco.MjModel.from_xml_string(scene_xml(scenario, obstacle_x))
        self.data = mujoco.MjData(self.model)
        self.body = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "uav")
        self.hull = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "uav_hull")
        self.data.qvel[0] = speed
        self.collision = False
        mujoco.mj_forward(self.model, self.data)

    def ranges(self):
        """从机体中心发射九束射线并排除自身。 / Cast nine beams from the body center, excluding the UAV."""
        origin = self.data.xpos[self.body].copy()
        values = []
        for angle in ANGLES:
            direction = np.array([np.cos(angle), np.sin(angle), 0.0])
            geom_id = np.array([-1], dtype=np.int32)
            distance = mujoco.mj_ray(self.model, self.data, origin, direction, None, 1, self.body, geom_id)
            values.append(MAX_RANGE if distance < 0 else min(distance, MAX_RANGE))
        return np.asarray(values)

    def step(self, target_speed, lateral_speed=0.0):
        """限幅加速度与高度保持；碰撞状态跨步保留。 / Limit acceleration, hold altitude and latch contacts."""
        if self.scenario == "crossing":
            self.data.mocap_pos[0, 1] = 2.5 - self.crossing_speed * self.data.time
        velocity = self.data.qvel[:3]
        acceleration = np.clip(3.0 * (float(target_speed) - velocity[0]), -2.0, 2.0)
        self.data.qfrc_applied[:] = [MASS * acceleration,
                                    MASS * np.clip(3 * (lateral_speed - velocity[1]), -2, 2),
                                    MASS * (9.81 - 18 * self.data.qpos[2] - 6 * velocity[2])]
        mujoco.mj_step(self.model, self.data)
        self.collision |= any(self.hull in (c.geom1, c.geom2) for c in self.data.contact)

    @property
    def position(self):
        return self.data.xpos[self.body].copy()


def expert_target(ranges, speed, cruise_speed):
    """用制动距离给出监督标签。 / Derive supervision from stopping distance."""
    gap = max(0.0, float(np.min(ranges[3:6])) - RADIUS - 0.7)
    # 预留反应时间与安全间距。 / Reserve reaction time and clearance.
    stopping = max(speed, 0.0) ** 2 / 4.0 + max(speed, 0.0) * 0.25
    risk = 2 if gap <= stopping else (1 if gap <= stopping + 1.5 else 0)
    safe_speed = max(0.0, np.sqrt(4 * gap + 0.25) - 0.5)
    return min(float(cruise_speed), safe_speed), risk
