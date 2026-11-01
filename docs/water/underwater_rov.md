水下无人机（ROV）MuJoCo 的建图导航 + 神经网络感知规划控制

原项目结构：model.py（MJCF 生成）→ sim.py（仿真内核 + 动力学）→ sensors.py（传感器）→ mapping.py（SLAM 占据栅格 / 扫描匹配）→ planning.py（A*+ 纯跟踪）→ nn.py（纯 numpy MLP/CNN、Adam）→ neural.py（策略封装）→ nav.py（导航主栈）+ ROS 节点 + 训练 / 评测脚本
水下版本命名：underwater_nn_nav，水下载体：ROV 水下机器人（带推进器，6 自由度，浮力 / 阻力 / 附加质量）

## 运行环境

- Ubuntu 20.04

- ROS Noetic

- Python 3.8


text
underwater_nn_nav/
├── model.py          # MJCF ROV模型、水下场景生成（礁石、管道、海底、水流）
├── sim.py            # ROV仿真内核、水动力、推进器力力矩计算
├── sensors.py        # 成像声呐、DVL、压力深度计、IMU
├── mapping.py        # 占据栅格SLAM、声呐扫描匹配（和原mapping几乎通用，只改射线）
├── planning.py       # A*代价地图、膨胀、纯跟踪（直接复用）
├── nn.py             # 纯numpy MLP/CNN、Adam、mse loss（**完全复用**）
├── neural.py         # 神经去噪、局部规划网络、控制器、端到端声呐网络
├── nav.py            # 导航主栈：SLAM -> A* / neural / e2e -> 速度指令输出
├── flight.py（改名hydro.py） # 水动力控制器，深度、姿态、速度闭环
├── scripts/
│   ├── sim_node.py       # ROS仿真节点，发布声呐、DVL、IMU、真值
│   ├── teleop_node.py    # 键盘遥控ROV（前后左右、上下、转向）
│   ├── nav_node.py       # 经典SLAM+A*导航ROS节点
│   ├── e2e_node.py       # 端到端声呐CNN节点
│   └── metrics_node.py   # ATE、碰撞、航程评测
├── tools/
│   ├── collect_data.py   # 采集声呐+导航标签数据集
│   ├── train_nets.py     # 训练4个网络：声呐去噪、规划、控制、端到端CNN
│   ├── evaluate.py       # 多场景benchmark评测
│   └── record_gif.py      # 录仿真演示
├── tests/
│   ├── run_all.py
│   ├── test_hydro.py
│   ├── test_mapping.py
│   ├── test_planning.py
│   └── test_nn.py
├── launch/
│   └── main.launch       # teleop / nav / neural / e2e
├── models/               # 保存npz权重：sonar_denoise.npz、rov_planner.npz、rov_controller.npz、e2e_sonar.npz
├── CMakeLists.txt
├── package.xml
├── requirements.txt
└── main.py               

关键模块代码
1. model.py （MuJoCo MJCF 水下 ROV 与场景）
重点：
水体没有在 MuJoCo 里做流体有限元（太复杂，课程项目用等效水动力模型，在 sim 层施加阻力、浮力，MuJoCo 只做刚体碰撞）
载体是 ROV，四个水平推进器（前后左右）+ 垂直推进器（控制深度）
场景：海底平面、礁石障碍物、管道；禁止 ROV 穿出水面（z 上限约束）
python
"""MuJoCo 水下ROV模型构建，中性浮力，推进器用xfrc_applied施力。
坐标约定：x前，y左，z向下（水下常用，z增大=更深）
"""
from __future__ import annotations
import math
import random
from dataclasses import dataclass, field
from typing import List, Tuple

# ROV参数
MASS = 1.8       # kg
BUOYANCY = 1.78  # 浮力，接近中性浮力，轻微负浮力
ARM_HORIZ = 0.18
INERTIA = (0.008, 0.008, 0.014)
MAX_THRUST_H = 2.2
MAX_THRUST_V = 1.2
ROV_HALF = (0.14, 0.12, 0.08)
DEFAULT_DEPTH = 1.5

@dataclass
class ReefObstacle:
    x: float
    y: float
    half_x: float
    half_y: float
    z_top: float = 0.3
    z_bottom: float = 3.0
    rgba: str = "0.25 0.28 0.32 1"

@dataclass
class UnderwaterScene:
    name: str
    size_x: float
    size_y: float
    start: Tuple[float, float, float] # x,y,yaw
    goal: Tuple[float, float]
    obstacles: List[ReefObstacle] = field(default_factory=list)
    seed: int = 0

def rov_xml(px:float, py:float, pz:float, yaw:float=0.0) -> str:
    extra = ""
    if yaw:
        extra = f' quat="{math.cos(yaw/2):.6f} 0 0 {math.sin(yaw/2):.6f}"'
    body = """
<body name="rov" pos="{px:.4f} {py:.4f} {pz:.4f}"{extra}>
  <freejoint name="root"/>
  <geom name="hull" type="box" size="{hx} {hy} {hz}" mass="{mass}" rgba="0.12 0.30 0.55 1" group="3"/>
  <site name="sonar" pos="0 0 0" size="0.02"/>
  <site name="rov" pos="0 0 0"/>
</body>
"""
    return body.format(px=px,py=py,pz=pz,extra=extra,
                       hx=ROV_HALF[0],hy=ROV_HALF[1],hz=ROV_HALF[2],mass=MASS)

def _obstacles(layout: UnderwaterScene) -> str:
    parts=[]
    for i,o in enumerate(layout.obstacles):
        parts.append(
            f'<geom name="reef_{i}" type="box" pos="{o.x:.4f} {o.y:.4f} {(o.z_top+o.z_bottom)/2:.4f}" '
            f'size="{o.half_x:.4f} {o.half_y:.4f} {(o.z_bottom-o.z_top)/2:.4f}" rgba="{o.rgba}"/>'
        )
    return "\n".join(parts)

def build_xml(layout: UnderwaterScene) -> str:
    sx,sy = layout.size_x, layout.size_y
    walls = f"""
<geom name="wall_n" type="box" pos="0 {sy/2:.3f} 1.6" size="{sx/2+0.2:.3f} 0.08 1.6"/>
<geom name="wall_s" type="box" pos="0 {-sy/2:.3f} 1.6" size="{sx/2+0.2:.3f} 0.08 1.6"/>
<geom name="wall_e" type="box" pos="{sx/2:.3f} 0 1.6" size="0.08 {sy/2:.3f} 1.6"/>
<geom name="wall_w" type="box" pos="{-sx/2:.3f} 0 1.6" size="0.08 {sy/2:.3f} 1.6"/>
"""
    return f"""
<mujoco model="underwater_nn_nav">
<compiler angle="radian" autolimits="true"/>
<option timestep="0.002" integrator="implicitfast" gravity="0 0 9.81"/>
<visual>
  <global offwidth="960" offheight="720"/>
  <headlight diffuse="0.4 0.4 0.4" ambient="0.25 0.25 0.25"/>
</visual>
<worldbody>
<geom name="seabed" type="plane" size="{max(sx,sy)/2+2:.2f} {max(sx,sy)/2+2:.2f} 0.05" rgba="0.18 0.20 0.22 1"/>
<site name="goal" pos="{layout.goal[0]:.4f} {layout.goal[1]:.4f} 1.5" size="0.12" rgba="0.1 0.9 0.6 1"/>
{walls}
{_obstacles(layout)}
{rov_xml(layout.start[0], layout.start[1], DEFAULT_DEPTH, layout.start[2])}
</worldbody>
</mujoco>
"""

# 场景工厂
def _reef_corridor(seed:int):
    rng = random.Random(seed+11)
    obs=[]
    for i in range(3):
        x = -1.0 + i*1.3
        side = 1 if i%2==0 else -1
        obs.append(ReefObstacle(x, side*1.0,0.14,0.45))
        obs.append(ReefObstacle(x, -side*1.0,0.14,0.45))
    return UnderwaterScene("reef_corridor",6.2,3.2,(-2.1,0.0,0.0),(2.1,0.0),obs,seed)

_SCENES = {"reef_corridor":_reef_corridor}
def make_layout(scene:str,seed=0):
    return _SCENES[scene](seed)
def scene_names():
    return tuple(_SCENES.keys())
2. sim.py 水动力仿真（替换原 DroneSim）
核心：浮力、阻力、附加质量、水流扰动，不再是无人机的升力模型；hydro.py 是控制器，输入目标机体系速度 
v 
x
​
 ,v 
y
​
 ,v 
z
​
 ,ω 
z
​
 
，输出推进器力力矩，通过xfrc_applied施加到 ROV 刚体。
python
"""ROV水下仿真内核：水动力 + 推进器控制 + 声呐/DVL传感器广播"""
from __future__ import annotations
import math
import numpy as np
import mujoco
from . import model as M
from .hydro import HydroController

class RovSim:
    def __init__(self, scene="reef_corridor", seed=0, render=False):
        self.layout = M.make_layout(scene,seed)
        self.model = mujoco.MjModel.from_xml_string(M.build_xml(self.layout))
        self.data = mujoco.MjData(self.model)
        self.model.opt.timestep = 0.002
        self.body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "rov")
        self.sonar_site = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "sonar")
        self.ctrl = HydroController(mass=M.MASS, buoyancy=M.BUOYANCY)
        self.cmd = {"vx":0.0,"vy":0.0,"vz":0.0,"yaw_rate":0.0,"depth_ref":1.5}
        self.renderer = None
        self._want_render = render
        self.reset()

    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        sx,sy,syaw = self.layout.start
        self.data.qpos[0:3] = [sx,sy,M.DEFAULT_DEPTH]
        self.data.qpos[3:7] = [math.cos(syaw/2),0,0,math.sin(syaw/2)]
        mujoco.mj_forward(self.model, self.data)
        self.ctrl.reset()
        # 稳定悬停
        for _ in range(300):
            self._physics_step()

    def set_command(self, vx=0.0, vy=0.0, vz=0.0, yaw_rate=0.0, depth_ref=None):
        self.cmd["vx"] = np.clip(vx,-1.2,1.2)
        self.cmd["vy"] = np.clip(vy,-1.2,1.2)
        self.cmd["vz"] = np.clip(vz,-0.6,0.6)
        self.cmd["yaw_rate"] = np.clip(yaw_rate,-1.0,1.0)
        if depth_ref is not None:
            self.cmd["depth_ref"] = np.clip(depth_ref,0.4,3.5)

    def _physics_step(self):
        pos = self.data.qpos[0:3]
        quat = self.data.qpos[3:7]
        vel = self.data.qvel[0:3]
        omega = self.data.qvel[3:6]
        # 水动力控制器输出合力、合力矩
        force, torque = self.ctrl.compute(pos, quat, vel, omega,
            self.cmd["vx"],self.cmd["vy"],self.cmd["vz"],self.cmd["yaw_rate"],self.cmd["depth_ref"])
        self.data.xfrc_applied[self.body_id,0:3] = force
        self.data.xfrc_applied[self.body_id,3:6] = torque
        mujoco.mj_step(self.model, self.data)
3. sensors.py：声呐（替换激光雷达）
原Lidar改成Sonar，射线逻辑几乎一样，只是水下声呐有衰减噪声、最大测距，不再是激光。DVL 输出机体速度（替代原来的里程计），压力计测深度。
python
class Sonar:
    """2D成像声呐，水平面扫描，替代激光雷达"""
    def __init__(self, model, n_beams=180, max_range=7.0, site="sonar", noise=0.04):
        self.model = model
        self.site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site)
        self.n = n_beams
        self.max_range = max_range
        self.noise = noise
        self.angles = np.linspace(-math.pi, math.pi, self.n, endpoint=False)
        self.geomgroup = np.array([1,1,1,0,0,0], dtype=np.uint8)
    def scan(self, data, rng=None):
        # 和Lidar几乎完全一致，噪声更大
        p = data.site_xpos[self.site_id]
        r = data.site_xmat[self.site_id].reshape(3,3)
        fwd = r @ np.array([1,0,0])
        yaw = math.atan2(fwd[1],fwd[0])
        out = np.empty(self.n)
        for i,a in enumerate(self.angles):
            ang = yaw + a
            vec = np.array([math.cos(ang), math.sin(ang), 0.0])
            d = mujoco.mj_ray(self.model, data, p, vec, self.geomgroup, 1, -1, np.zeros(1,dtype=np.int32))
            out[i] = d if d>=0 else self.max_range
        if self.noise>0 and rng is not None:
            out += rng.normal(0,self.noise,size=self.n)
        return np.clip(out,0.05,self.max_range)
4. mapping.py 直接复用
扫描匹配、OccupancyGrid、Slam类几乎不用改！
输入从Lidar.ranges换成Sonar.ranges
只需要在Slam.update()里传入声呐角度，栅格更新逻辑不变，2D 平面 SLAM 完全通用。
5. planning.py 完全复用
A*、CostMap 膨胀、PurePursuit 纯跟踪，不需要改动；导航目标是平面 xy，深度单独控制。
6. neural.py & nn.py 几乎复用
四个网络，只改输入输出：
声呐去噪网络：输入滑窗声呐读数，输出干净声呐距离（替代激光去噪）
局部规划网络：局部占据栅格 patch + 目标相对坐标，输出前进方向
控制器网络：距离、航向误差、声呐分扇区净空、上一时刻速度，输出 
(v 
x
​
 ,v 
y
​
 ,v 
z
​
 ,ω 
z
​
 )
端到端 CNN：输入声呐灰度图（声呐扇区图像），输出机体系速度指令
7. nav.py 导航栈
改动点：
增加深度闭环控制，维持目标深度
控制输出多了 vz（垂直速度）
碰撞检测：礁石、海底、边界墙
模式：teleop /classical（SLAM+A*） /neural 规划 /e2e 声呐端到端

evaluate.py：
评测指标：成功率、航行时间、航程、ATE（SLAM 位姿 RMSE）、碰撞次数、深度保持误差

ROS 节点、launch、main.py
sim_node.py：发布/sonar（替代/scan）、/dvl、/rov/depth、IMU、真值位姿
nav_node.py：订阅声呐，跑 SLAM+A*，发布/cmd_vel（twist，linear.z 是垂直速度）
teleop_node.py：按键增加上升 / 下降
main.py无 ros 入口，mode 参数：teleop / nav / neural / e2e / test / demo
