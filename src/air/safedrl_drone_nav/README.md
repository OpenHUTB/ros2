# SafeDRL-DroneNav-ROS

**Simulator-neutral control-barrier-function safety layer for learned drone navigation**

This package provides a bottom-layer CBF safety controller and a ROS node that can be
moved between the course simulators without retraining or rewriting the policy.  The
navigator exchanges only standard ROS messages:

| Topic | Type | Direction | Frame |
|---|---|---|---|
| `/drone/odom` | `nav_msgs/Odometry` | simulator -> navigator | ENU world / FLU body |
| `/drone/scan` | `sensor_msgs/LaserScan` | simulator -> navigator (optional) | `base_link` |
| `/drone/cmd_vel` | `geometry_msgs/Twist` | navigator -> simulator | ENU / FLU |
| `/drone/nav_status` | `std_msgs/String` | navigator -> diagnostics | JSON |

Course simulator adapters:

| Simulator | Adapter | Notes |
|---|---|---|
| **AirSim** | `scripts/airsim_bridge_node.py` + `launch/airsim_navigate.launch` | `airsim_ros_pkgs` backend or direct `airsim` Python API; NED/FRD <-> ENU/FLU conversion |
| **MuJoCo** | `scripts/mujoco_bridge_node.py` + `models/quadrotor_scene.xml` + `launch/mujoco_navigate.launch` | rigid-body plant, first-order velocity loop, ray-cast `LaserScan` |
| PyBullet | `scripts/safe_nav_env.py` | offline unit-level mathematical verification only, not the course simulator |

The policy is a constrained (Lagrangian) PPO actor trained in the lightweight PyBullet
verification environment.  The CBF filter and the observation builder are shared
verbatim between training, the AirSim adapter and the MuJoCo adapter, so the runtime
safety layer does not depend on the simulator that produced the checkpoint.

> **完整中文文档（图表、实验分析与复现步骤）**：
> [基于 CBF-PPO 的无人机安全导航](../../../docs/air/safedrl_drone_nav.md)
>
> 报告图片只在仓库顶层的 `docs/img/safedrl_drone_nav/` 保存一份，功能包源码目录内
> 不重复存放静态图表。

---

## 1. Offline verification result

The following ablation was measured in the offline PyBullet environment with identical
policy weights and identical evaluation seeds (24 episodes).  The only variable is
whether the CBF filter is enabled at inference time.

| # | Policy | CBF filter | Success | Collision | Worst contact clearance | Mean arrival | Mean cost |
|---|--------|:----------:|:-------:|:---------:|:-----------------------:|:------------:|:---------:|
| 1 | PPO (unconstrained) | no | 0.458 | 0.542 | **-0.091 m** | 6.82 s | 0.546 |
| 2 | PPO-Lagrangian | no | 0.708 | 0.292 | **-0.083 m** | 14.77 s | 0.303 |
| 3 | PPO-Lagrangian | yes | **1.000** | **0.000** | **+0.175 m** | 15.93 s | **0.001** |
| 4 | PPO (unconstrained) | yes | 0.833 | 0.167 | **+0.137 m** | 7.65 s | 0.167 |

Clearance is the distance to actual contact; a negative value means the airframe
geometrically penetrated an obstacle.

1. **The filter removes contact.** Without it the policies penetrate obstacles.  With
   it, all 96 filtered episodes stay clear, and the worst case keeps 0.137 m.
2. **The filter also raises task success.** For the unconstrained policy, success goes
   from 0.458 to 0.833 because most unfiltered failures are avoidable crashes.

These numbers come from the offline plant.  They are the training/verification result,
not a claim about a particular AirSim or MuJoCo map; the adapter layer is what carries
the same controller to those simulators.

Figures 1-7 of the [course page](../../../docs/air/safedrl_drone_nav.md) show the 3-D
flight path, the clearance distribution, the ablation, the state response and the
training curves.

---

## 2. Method

### 2.1 Offline verification environment (`scripts/safe_nav_env.py`)

The offline environment exists to train the policy and to pin the safety mathematics
without requiring a GPU, Unreal or a MuJoCo installation.

* **Plant** — a CrazyFlie-2.X-class quadrotor (`scripts/drone_dynamics.py`): 27 g,
  explicit inertia, rotor mixer and a 20 ms motor lag.  The RL action is a desired
  velocity plus yaw rate; an SE(3) controller converts it to thrust and body torques.
* **Task** — thread three non-colinear gates in order.  The gate offsets form a slalom
  (y: 0.0 -> +1.55 -> -1.55 m; z: 0.0 -> +0.45 -> -0.35 m), so a constant forward
  command fails.
* **Disturbances** — an Ornstein-Uhlenbeck wind process plus discrete gust bursts.
* **Obstacles** — 7 static spheres and 4 dynamic spheres sweeping across the corridor.
* **Observation (53-D)** — relative gate position (3), velocity (3), attitude (3), four
  nearest obstacles as (relative position, radius) (16), a 24-ray virtual lidar and the
  previous action (4).
* **Action (4-D)** — desired world velocity (3, scaled to +/-2.6 m/s) plus yaw rate.

### 2.2 Reward and constrained RL (`scripts/lagrangian_ppo.py`)

Guidance uses potential-based reward shaping with `Phi(s) = -distance_to_active_gate`
(Ng, Harada & Russell 1999).  Energy is the rotor aerodynamic power `sum(rpm^3)`;
safety enters as a **cost** stream rather than a reward penalty:

```
cost = 1{collision} + 0.5 * (margin-encroachment depth) / max_steps    in [0, 1.5]
```

PPO-Lagrangian adds a cost-value head and updates the multiplier `lambda` by projected
dual ascent.  `cost_limit = 0.6` therefore reads as "at most a 60 % collision rate".

### 2.3 CBF safety filter (`scripts/safety.py`)

For each active barrier with value `h_i` and unit gradient `a_i`, the filter solves

```
v_safe = argmin ||v - v_des||^2   s.t.   a_i^T v >= b_i,   ||v|| <= v_max
```

A single constraint has a closed-form spherical-cap projection
(`project_halfspace_ball`); multiple constraints use a short Gauss-Seidel sweep.

The discrete CBF condition `h(x+) >= (1-gamma) h(x)` expands to
`grad_h . v >= -gamma h / dt`, which at 30 Hz is far too permissive for a
velocity-commanded quadrotor.  The implemented bound is a stopping-distance barrier:

```
v_approach <= ( -a*tau + sqrt((a*tau)^2 + 2*a*h) ) / brake_gain
```

which is the positive root of `v*tau + v^2/(2a) = h`.  The `a*tau` term accounts for
the first-order velocity lag of the plant.  The same equation is used by the MuJoCo
plant (`tau_lag = 0.20 s`), so the certificate matches the adapter's dynamics.

Barriers cover obstacles, the ground, the ceiling and the corridor walls.  Gate frames
are not barriers: the task requires flying through the aperture.

### 2.4 Behaviour-cloning warm start (`scripts/pretrain_bc.py`)

PPO from a random initialisation falls into a hovering local optimum on this task.
A pure-pursuit expert is cloned first (200 episodes, 250 epochs), then PPO fine-tunes
the actor with a small learning rate.  All ablation rows share the same warm start, so
the comparison isolates the safety mechanism.

---

## 3. ROS integration

| Node | Role |
|---|---|
| `scripts/drl_navigator_node.py` | policy + CBF filter; consumes odometry and optionally a laser scan, publishes `Twist` and JSON status |
| `scripts/airsim_bridge_node.py` | AirSim adapter (ROS wrapper or direct Python API), including NED/FRD <-> ENU/FLU conversion |
| `scripts/mujoco_bridge_node.py` | MuJoCo rigid-body plant, velocity loop, odometry and ray-cast scan |
| `scripts/dummy_state_publisher.py` | simulator-free point-mass plant for the offline ROS smoke test |
| `scripts/sim_frames.py` | shared frame conversions used by the AirSim adapter and its tests |

The navigator has two barrier sources:

* `~obstacles` — the 3-D sphere list used by the offline evaluation;
* `~scan_barriers` — build 2-D barriers from the live `LaserScan`; enabled by the
  AirSim and MuJoCo launch files.

### AirSim

```bash
# terminal 1: Unreal + AirSim vehicle, and (for backend:=ros) the ROS wrapper
roslaunch airsim_ros_pkgs airsim_node.launch

# terminal 2: adapter + navigator
roslaunch safedrl_drone_nav airsim_navigate.launch backend:=ros
# or, without airsim_ros_pkgs:
roslaunch safedrl_drone_nav airsim_navigate.launch backend:=api
```

Gate and obstacle coordinates in the launch file are relative to the AirSim vehicle
start point.  If `lidar_topic` is set, the adapter flattens the PointCloud2 stream into
the standard `LaserScan`; otherwise the navigator uses the parameter-server obstacle
list for the observation features and falls back to the 3-D barrier terms.

### MuJoCo

```bash
roslaunch safedrl_drone_nav mujoco_navigate.launch
roslaunch safedrl_drone_nav mujoco_navigate.launch use_cbf:=false   # ablation
```

The MJCF scene (`models/quadrotor_scene.xml`) contains the free-flying quadrotor, the
corridor walls, four static obstacles and visual-only gate frames.  Gates are in geom
group 2, so they are drawn but excluded from the ray caster and from contact.

### Offline smoke test (no course simulator)

```bash
roslaunch safedrl_drone_nav smoke_test.launch
roslaunch safedrl_drone_nav navigate.launch \
    model_path:=$(rospack find safedrl_drone_nav)/checkpoints/best_model.zip
```

---

## 4. Repository layout

```text
safedrl_drone_nav/
├── CMakeLists.txt
├── package.xml
├── README.md
├── requirements.txt            # training + offline verification
├── requirements-sim.txt        # optional airsim / mujoco adapters
├── config/
│   ├── nav_params.yaml         # offline environment and training parameters
│   └── interface_params.yaml   # standard topics and frame conventions
├── models/
│   └── quadrotor_scene.xml     # MuJoCo plant
├── launch/
│   ├── airsim_navigate.launch  # course simulator: AirSim
│   ├── mujoco_navigate.launch  # course simulator: MuJoCo
│   ├── navigate.launch         # offline dummy plant
│   ├── smoke_test.launch       # communication self-test
│   └── train.launch            # training / evaluation wrapper
├── scripts/
│   ├── nav_config.py           # shared task/observation configuration (no simulator deps)
│   ├── sim_frames.py           # NED/FRD <-> ENU/FLU
│   ├── airsim_bridge_node.py
│   ├── mujoco_bridge_node.py
│   ├── drl_navigator_node.py
│   ├── dummy_state_publisher.py
│   ├── safety.py               # CBF filter and Lagrange dual
│   ├── drone_dynamics.py       # offline quadrotor
│   ├── safe_nav_env.py         # offline Gymnasium environment
│   ├── lagrangian_ppo.py       # constrained PPO
│   ├── pretrain_bc.py
│   ├── train.py
│   ├── evaluate.py
│   ├── make_report_figures.py
│   ├── figures.py
│   └── run_logging.py
├── tests/
│   ├── test_bridge_frames.py   # frame conversions, MJCF, interface spec
│   ├── test_safety_filter.py   # analytic projection tests
│   └── test_environment.py     # offline closed-loop tests
└── checkpoints/
    └── best_model.zip          # the only weight file in git
```

Report figures are kept once under the repository-level
`docs/img/safedrl_drone_nav/`; this package has no `docs/` directory.

---

## 5. Reproducing the offline result

```bash
conda create -n safedrl python=3.10 -y && conda activate safedrl
pip install -r requirements.txt

python tests/test_safety_filter.py      # 11 analytic projection tests
python tests/test_environment.py        # 8 offline closed-loop tests
python tests/test_bridge_frames.py      # 10 interface tests (no simulator needed)

python scripts/pretrain_bc.py --use-cbf --episodes 200 --epochs 250 \
    --out-name bc_warmstart_cbf
python scripts/train.py --policy ppo_lag --use-cbf \
    --init-from checkpoints/bc_warmstart_cbf.zip --normalize-reward \
    --timesteps 90000 --run-name ppo_lagrangian --seed 12 --max-steps 700
python scripts/evaluate.py --model checkpoints/best_model.zip --use-cbf \
    --run-name ppo_lagrangian_cbf --episodes 24 --figures
python scripts/make_report_figures.py \
    --policy ppo_baseline --policy ppo_lagrangian \
    --policy ppo_lagrangian_cbf --policy ppo_baseline_cbf --tag ablation
```

PyBullet publishes no Windows wheel on PyPI.  On Windows, use the conda-forge build:

```bash
conda install -c conda-forge pybullet "numpy<2"
```

For the course simulators:

```bash
pip install -r requirements-sim.txt     # airsim + mujoco
```

---

## 6. Tests

* `tests/test_bridge_frames.py` — 10 tests.  Pins NED/FRD -> ENU/FLU position,
  velocity, attitude and yaw-rate conversions, the PointCloud2 -> LaserScan
  projection, the MJCF scene structure and the interface specification.  Runs without
  ROS, AirSim, MuJoCo or PyBullet.
* `tests/test_safety_filter.py` — 11 analytic tests of the spherical-cap projection,
  discrete-CBF enforcement, saturation branches and dual-ascent monotonicity.
* `tests/test_environment.py` — 8 closed-loop tests in the offline plant, including
  the adversarial-policy contact test that backs the headline result.

---

## 7. Limitations

* **Gate-frame contacts are not barriers.**  The barrier set covers obstacles, ground,
  ceiling and walls.  Gate frames stay flyable by design; residual grazes are caught by
  the cost signal, not by the filter.
* **`lambda` stayed at zero in the successful row.**  The filter already removed the
  violations, so the dual variable had nothing to penalise.  The effect of the
  Lagrangian term is visible in row 2 of the table.
* **The filter is conservative.**  It costs arrival time (6.82 s -> 15.93 s) because it
  assumes 3.0 m/s^2 braking authority, below the controller's ~7.5 m/s^2 capability.
* **Single training seed.**  Arrival-time differences are not statistically established.
* **Bridge validation scope.**  The frame conversions and the MJCF scene are covered by
  unit tests, but a live AirSim or MuJoCo flight requires the course simulator and is
  not part of the offline checks.  The reported ablation numbers come from the offline
  PyBullet plant.

---

## 8. References

1. Ng, Harada & Russell (1999). *Policy invariance under reward transformations.* ICML.
2. Ames, Coogan, Egerstedt, Notomista, Sreenath & Tabuada (2019). *Control barrier
   functions: theory and applications.* ECC.
3. Agrawal & Sreenath (2017). *Discrete control barrier functions for safety-critical
   control of discrete systems.* ACC.
4. Stooke, Achiam & Abbeel (2020). *Responsive safety in reinforcement learning by PID
   Lagrangian methods.* ICML.
5. Lee, Leok & McClamroch (2010). *Geometric tracking control of a quadrotor UAV on
   SE(3).* CDC.
6. Schulman, Wolski, Dhariwal, Radford & Klimov (2017). *Proximal policy optimization
   algorithms.* arXiv:1707.06347.

## 9. License

MIT.  The environment design is inspired by
[`eRGiBi/DRL-DroneNavigation`](https://github.com/eRGiBi/DRL-DroneNavigation) (MIT); the
dynamics, safety filter, constrained-RL formulation, ROS interface and simulator
adapters are an independent re-implementation.
