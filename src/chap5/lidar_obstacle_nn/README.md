# lidar_obstacle_nn — Gazebo LiDAR Obstacle Classification (ROS 2 Humble)

A small neural-network module that classifies a 2D LiDAR scan from a
Gazebo Fortress simulation into three obstacle-proximity categories:

| label | class | meaning |
| --- | --- | --- |
| `0` | `near` | Closest obstacle inside the danger radius (default 1.0 m). |
| `1` | `far`  | Obstacle visible but outside the danger radius. |
| `2` | `free` | No obstacle within the scan range. |

It satisfies the OpenHUTB/ros2 contribution rules:
- launch startup (`launch/main.launch.py`)
- `main.py` entry point
- all-lowercase, underscore-separated file names
- no binary blobs larger than 3 MB (the trained MLP is a few hundred KB)
- README with environment + run steps

## 1. Environment

| Item | Version |
| --- | --- |
| OS | Ubuntu 22.04 LTS |
| ROS | ROS 2 Humble (`/opt/ros/humble`) |
| Simulator | Gazebo Fortress 6.x (Ignition Gazebo) |
| Python | 3.10 (the one shipped with ROS 2 Humble) |
| GPU (optional) | NVIDIA GPU + CUDA 12.x (CPU-only machines work fine) |

Python dependencies (install into the ROS 2 Python, **not** a
separate venv, so that `rclpy` stays importable):

```bash
source /opt/ros/humble/setup.bash
python3 -m pip install --user -r src/chap5/lidar_obstacle_nn/requirements.txt
```

**CPU-only machines:** the default PyPI build of `torch` already runs
on CPU — nothing extra to install. The inference node detects this and
runs on CPU (CPU-only torch builds raise `AssertionError` on any CUDA
call; the node skips the CUDA probe entirely in that case).

**Machines with an NVIDIA GPU:** install the CUDA build of `torch`
matching your driver (see <https://pytorch.org/get-started/locally/>):

```bash
python3 -m pip install --user torch --index-url https://download.pytorch.org/whl/cu121
```

**Headless machines (no monitor):** the GPU lidar sensor needs a
rendering context, so install `xvfb` and wrap the launch command (see
section 3):

```bash
sudo apt install xvfb
```

Verify:

```bash
python3 -c "import torch; print(torch.__version__, torch.cuda.is_available())"
# e.g.  2.3.1+cpu False     (CPU-only machine — expected and supported)
```

## 2. Build

```bash
source /opt/ros/humble/setup.bash
cd /path/to/ros2_ws          # the folder containing this package under src/
colcon build --packages-select lidar_obstacle_nn --symlink-install
source install/setup.bash
```

## 3. Run (one-liner)

> **First run? Train first.** The trained weights (`models/lidar_mlp.pt`)
> are intentionally **not** committed (the repo ignores `*.pt` binaries
> to keep the repository text-only), so before the very first launch you
> must generate them once — see section 4 (about a minute on CPU).
> `inference_node` would warn `Model file not found` and fall back to a
> random-weight network without them.

With a display attached:

```bash
ros2 launch lidar_obstacle_nn main.launch.py
```

Without a display (server / VM / CI), wrap the command in a virtual X
server — the `gpu_lidar` sensor renders into it and `/scan` contains
real echoes:

```bash
xvfb-run -a ros2 launch lidar_obstacle_nn main.launch.py
```

This starts:
1. Gazebo Fortress with `worlds/obstacle_world.sdf` (a 360-beam LiDAR
   on a box-shaped robot, surrounded by 3 obstacles at different
   radii). Add `world:=dynamic` to load `obstacle_world_dynamic.sdf`
   instead (one obstacle on a controllable prismatic slide).
2. The `ros_gz_bridge` `parameter_bridge` forwarding Gazebo `/scan`
   to ROS 2 `/scan` (`sensor_msgs/LaserScan`). In the dynamic world it
   also bridges `/obstacle_slider/cmd_pos` (`std_msgs/Float64` ->
   `ignition.msgs.Double`) used by the data collector.
3. The `inference_node` that loads the trained MLP and publishes
   `/obstacle_class` (`std_msgs/String`) and
   `/obstacle_confidence` (`std_msgs/Float32`).

Inspect the output in a second terminal:

```bash
ros2 topic echo /obstacle_class
ros2 topic echo /obstacle_confidence
ros2 topic hz /scan
```

### Why not `ign gazebo --headless-rendering`?

Measured on Ubuntu 22.04 + Gazebo Fortress 6.18.0 with this world:

| Launch method | `/scan` beams hitting an obstacle | Verdict |
| --- | --- | --- |
| `ign gazebo -r -s --headless-rendering` | 0/360 (every beam = 30 m max range) | no echoes — dataset collection and demo are meaningless |
| `xvfb-run -a` + normal GUI build | 76/360 (closest 0.40 m) | works — matches the scene |

`--headless-rendering` gives the `gpu_lidar` sensor no rendering
context, so it never produces echoes. Always use `xvfb-run -a` (or a
real display) for data collection and inference.

## 4. Train your own model

### 4.1 Collect a labelled dataset

The default, Gazebo-free path is the synthetic generator (class-balanced,
about a minute — shown in 4.2 below). It is what the numbers in this
README and the docs page were produced with.

#### 4.1.1 Balanced real data from the dynamic world

`obstacle_world_dynamic.sdf` puts the obstacle on a horizontal
prismatic joint driven by Gazebo's `JointPositionController`. The
collector publishes the joint position and visits six stations (two
distances per class), accepting a scan only when its label matches the
station **and** the measured distance is within a configurable band,
so the resulting dataset is balanced by construction:

```bash
# Terminal A: dynamic world + bridges (add xvfb-run -a without a display)
ros2 launch lidar_obstacle_nn sim.launch.py world:=dynamic

# Terminal B: 150 scans x 6 stations = 900 scans (300 per class),
# fully automatic, about 4 minutes
ros2 run lidar_obstacle_nn collect_data --ros-args \
    -p motion_enabled:=true \
    -p per_station:=150 \
    -p output_path:=/tmp/lidar_real.npz
```

Train on it and evaluate any checkpoint (accuracy, per-class metrics,
confusion matrix):

```bash
python3 -m lidar_obstacle_nn.train_model \
    --dataset /tmp/lidar_real.npz --output /tmp/lidar_real.pt \
    --epochs 50 --device cpu
python3 -m lidar_obstacle_nn.evaluate_model \
    --model /tmp/lidar_real.pt --dataset /tmp/lidar_real.npz
```

Measured on the same held-out real set (180 scans, 60 per class):
the model trained on 720 real scans scores **100%**, while the
synthetic-data model scores **83.9%** — it misses 29/60 `far` scans
(recall 51.7%) because synthetic wedge geometry differs from the real
box returns. The synthetic generator is still the fastest way to
validate the pipeline; the dynamic world closes the sim-to-real gap.

#### 4.1.2 Static world collection (single class)

Collecting from the static world also works, but yields one class:

- `lidar_box` in `obstacle_world.sdf` is `static` with **no
  velocity/diff-drive plugin**, and a 360 degree scan always sees the
  nearest box no matter how the robot rotates. Standing still, the
  auto-labeller assigns a single class (measured: 20/20 frames all
  `near`). Use the dynamic world above for all three classes.
- Collection needs a rendering context: use a real display or
  `xvfb-run -a ros2 launch lidar_obstacle_nn sim.launch.py`.

```bash
# Terminal A: start the static simulation
ros2 launch lidar_obstacle_nn sim.launch.py

# Terminal B: collect 1000 scans (all "near" in the default scene)
ros2 run lidar_obstacle_nn collect_data --ros-args \
    -p output_path:=/tmp/lidar_dataset.npz \
    -p num_samples:=1000
```

### 4.2 Train the MLP

```bash
# 1. Synthetic dataset (no Gazebo needed), class-balanced
python3 -m lidar_obstacle_nn.generate_synthetic_dataset \
    --output /tmp/lidar_dataset.npz --num-samples 1500

# 2. Train
python3 -m lidar_obstacle_nn.train_model \
    --dataset /tmp/lidar_dataset.npz \
    --output  src/chap5/lidar_obstacle_nn/models/lidar_mlp.pt \
    --epochs 50
```

Training a 128-hidden-unit MLP for 50 epochs takes seconds on CPU.
The best validation weights are saved to `models/lidar_mlp.pt` and a
metrics sidecar to `models/lidar_mlp.pt.json` (regenerated on every
training run; not committed to the repository).

### 4.3 Rebuild so the new weights ship with the package

```bash
colcon build --packages-select lidar_obstacle_nn --symlink-install
source install/setup.bash
ros2 launch lidar_obstacle_nn main.launch.py
```

## 5. Files

| Path | Purpose |
| --- | --- |
| `package.xml` | ROS 2 ament_python package manifest. |
| `setup.py` / `setup.cfg` | Python install scripts + entry points. |
| `lidar_obstacle_nn/` | Python package (sources). |
| `lidar_obstacle_nn/main.py` | Documented entry point (calls inference_node). |
| `lidar_obstacle_nn/model.py` | `LidarMLP` PyTorch module. |
| `lidar_obstacle_nn/inference_node.py` | ROS 2 node wrapping the MLP. |
| `lidar_obstacle_nn/collect_data.py` | Records `/scan` + auto-labels into `.npz`; motion mode drives the dynamic-world slide through labelled stations. |
| `lidar_obstacle_nn/train_model.py` | Trains the MLP and saves `.pt` weights. |
| `lidar_obstacle_nn/evaluate_model.py` | Accuracy, per-class precision/recall and confusion matrix on a labelled `.npz`. |
| `launch/main.launch.py` | Top-level launch entry (`world:=static/dynamic`). |
| `launch/sim.launch.py` | Gazebo + ros_gz_bridge; also bridges the slide command in the dynamic world. |
| `worlds/obstacle_world.sdf` | Static Gazebo world (robot + 3 obstacles + LiDAR). |
| `worlds/obstacle_world_dynamic.sdf` | Dynamic Gazebo world: one obstacle on a position-controlled prismatic slide. |
| `config/params.yaml` | Default ROS 2 parameters. |
| `models/` | Trained weights (`lidar_mlp.pt`, generated locally). |
| `requirements.txt` | Python deps. |

## 6. Topics

| Topic | Type | Direction |
| --- | --- | --- |
| `/scan` | `sensor_msgs/LaserScan` | input (from `ros_gz_bridge`) |
| `/obstacle_class` | `std_msgs/String` | output (`near` / `far` / `free`) |
| `/obstacle_confidence` | `std_msgs/Float32` | output (max softmax probability) |
| `/obstacle_slider/cmd_pos` | `std_msgs/Float64` | output (dynamic world only; collector -> slide joint position) |

## 7. Notes / Limitations

- The MLP is intentionally tiny (360-128-128-3 ≈ 50 K parameters,
  ~200 KB on disk) so the binary stays well under the 3 MB guidance.
- `inference_node.py` falls back to a geometric rule (min range vs
  the two thresholds) when the network confidence is below 0.45, so
  the module is still useful before training completes.
- Gazebo reports `inf` ranges when a beam misses; both the collector
  and the inference node replace those with the scan maximum (30 m),
  which the MLP treats as "no return".
- CUDA probing (`use_gpu:=true`) only does anything on a **CUDA build**
  of torch: it tries one forward pass on the first `/scan` and falls
  back to CPU if that fails (`RuntimeError`, e.g. a compute-capability
  mismatch). On a **CPU-only build** of torch the probe is skipped
  entirely (CPU-only builds raise `AssertionError` on any CUDA call,
  which is also caught as a safety net). Either way the node keeps
  running on CPU instead of exiting.
- Data collection and inference need a **rendering context** for the
  `gpu_lidar` sensor: use a real display or `xvfb-run -a`. Plain
  `--headless-rendering` yields all-max-range scans (see section 3).
- In the static world `lidar_box` is `static` with no control plugin,
  so live collection there yields a single class. The dynamic world
  (`world:=dynamic`) automates balanced three-class collection; the
  synthetic generator remains the fastest no-Gazebo path.
- The dynamic slide controller's default Gazebo topic contains a `/0/`
  token, which is an illegal ROS 2 topic name; the SDF overrides it to
  `/obstacle_slider/cmd_pos`. Keep that in mind when renaming joints.
- A model trained only on synthetic data reached 83.9% on real sim
  scans (far-class recall 51.7%); training on dynamic-world data
  reached 100% on the same held-out set — see section 4.1.1 and the
  docs page for the full comparison.
