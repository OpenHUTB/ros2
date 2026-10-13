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
| GPU (optional) | NVIDIA RTX 5070 Ti with CUDA 12.x |

Python dependencies (install into the ROS 2 Python, **not** a
separate venv, so that `rclpy` stays importable):

```bash
source /opt/ros/humble/setup.bash
python3 -m pip install --user -r src/chap5/lidar_obstacle_nn/requirements.txt
```

For an NVIDIA GPU, install the CUDA build of `torch` manually so you
can pick the wheel matching your driver:

```bash
python3 -m pip install --user torch --index-url https://download.pytorch.org/whl/cu121
```

Verify:

```bash
python3 -c "import torch; print(torch.__version__, torch.cuda.is_available())"
# e.g.  2.3.1+cu121 True
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
> must generate them once — see section 4 (about 2 minutes on CPU).
> `inference_node` would warn `Model file not found` and fall back to a
> random-weight network without them.

```bash
ros2 launch lidar_obstacle_nn main.launch.py
```

This starts:
1. Gazebo Fortress with `worlds/obstacle_world.sdf` (a 360-beam LiDAR
   on a box-shaped robot, surrounded by 3 obstacles at different
   radii).
2. The `ros_gz_bridge` `parameter_bridge` forwarding Gazebo `/scan`
   to ROS 2 `/scan` (`sensor_msgs/LaserScan`).
3. The `inference_node` that loads the trained MLP and publishes
   `/obstacle_class` (`std_msgs/String`) and
   `/obstacle_confidence` (`std_msgs/Float32`).

Inspect the output in a second terminal:

```bash
ros2 topic echo /obstacle_class
ros2 topic echo /obstacle_confidence
ros2 topic hz /scan
```

## 4. Train your own model

### 4.1 Collect a labelled dataset

The Gazebo world contains obstacles at three different radii so that
rotating the robot past them is enough to sample all three classes.
The collector labels each scan automatically from its minimum range.

```bash
# Terminal A: start the simulation
ros2 launch lidar_obstacle_nn sim.launch.py

# Terminal B: collect 1000 scans
ros2 run lidar_obstacle_nn collect_data --ros-args \
    -p output_path:=/tmp/lidar_dataset.npz \
    -p num_samples:=1000

# (Optional) Move the robot around in the Gazebo GUI so all three
# obstacle classes appear in the scan.
```

### 4.2 Train the MLP

```bash
python3 -m lidar_obstacle_nn.train_model \
    --dataset /tmp/lidar_dataset.npz \
    --output  src/chap5/lidar_obstacle_nn/models/lidar_mlp.pt \
    --epochs 50
```

Training a 128-hidden-unit MLP for 50 epochs on a 5070 Ti takes
about 1-2 minutes. The best validation weights are saved to
`models/lidar_mlp.pt` and a metrics sidecar to
`models/lidar_mlp.pt.json`.

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
| `lidar_obstacle_nn/collect_data.py` | Records `/scan` + auto-labels into `.npz`. |
| `lidar_obstacle_nn/train_model.py` | Trains the MLP and saves `.pt` weights. |
| `launch/main.launch.py` | Top-level launch entry. |
| `launch/sim.launch.py` | Gazebo + ros_gz_bridge only. |
| `worlds/obstacle_world.sdf` | Gazebo world (robot + 3 obstacles + LiDAR). |
| `config/params.yaml` | Default ROS 2 parameters. |
| `models/` | Trained weights (`lidar_mlp.pt`). |
| `data/` | Optional example dataset (kept small). |
| `requirements.txt` | Python deps. |

## 6. Topics

| Topic | Type | Direction |
| --- | --- | --- |
| `/scan` | `sensor_msgs/LaserScan` | input (from `ros_gz_bridge`) |
| `/obstacle_class` | `std_msgs/String` | output (`near` / `far` / `free`) |
| `/obstacle_confidence` | `std_msgs/Float32` | output (max softmax probability) |

## 7. Notes / Limitations

- The MLP is intentionally tiny (360-128-128-3 ≈ 50 K parameters,
  ~200 KB on disk) so the binary stays well under the 3 MB guidance.
- `inference_node.py` falls back to a geometric rule (min range vs
  the two thresholds) when the network confidence is below 0.45, so
  the module is still useful before training completes.
- Gazebo reports `inf` ranges when a beam misses; both the collector
  and the inference node replace those with the scan maximum (30 m),
  which the MLP treats as "no return".
