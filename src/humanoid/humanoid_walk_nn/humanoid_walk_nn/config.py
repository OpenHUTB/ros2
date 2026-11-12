"""Shared constants and paths for the humanoid walking project (HNN)."""

from __future__ import annotations

import os

ROOT = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
MODELS_DIR = os.path.join(ROOT, "models")
RESULTS_DIR = os.path.join(ROOT, "results")
DOCS_DIR = os.path.join(ROOT, "docs")
ASSETS_DIR = os.path.join(DOCS_DIR, "assets")
MEDIA_DIR = os.path.join(ROOT, "media")
CONFIG_DIR = os.path.join(ROOT, "config")

# ------------------------------------------------------------------ mapping
MAP_RESOLUTION = 0.08        # m / cell, occupancy grid used by SLAM
PLAN_RESOLUTION = 0.20       # m / cell, coarse grid used by A*
ROBOT_RADIUS = 0.32          # humanoid footprint radius (m)
INFLATION_RADIUS = 0.55

# ------------------------------------------------------------- walking model
STEP_LENGTH = 0.36           # m per step
MAX_SPEED = 1.25             # m/s forward
MAX_BACKWARD = 0.50
MAX_ACCEL = 1.10             # m/s^2
MAX_DECEL = 1.60
MAX_TURN_RATE = 1.50         # rad/s
TURN_ACCEL = 3.20            # rad/s^2
BALANCE_LIMIT = 0.16         # m, lateral CoM offset
BALANCE_RESTORE = 3.20       # 1/s
BALANCE_GAIN = 0.45          # m per (m/s * rad/s)
FALL_HOLD = 0.45             # s beyond limit before a fall is registered
GOAL_TOLERANCE = 0.45        # m

# ------------------------------------------------------------------- sensors
LIDAR_BEAMS = 72
LIDAR_FOV = 6.2831853
LIDAR_RANGE_MIN = 0.15
LIDAR_RANGE_MAX = 8.00
CAM_WIDTH = 32
CAM_HEIGHT = 32
CAM_FOV = 1.0472             # 60 deg forward depth camera
CAM_RANGE = 8.0
GNSS_SIGMA = 0.20
IMU_SIGMA = 0.004

# ------------------------------------------------------------------- control
CONTROL_HZ = 20.0
PHYSICS_HZ = 100.0
PHYS_DT = 1.0 / PHYSICS_HZ
SENSOR_HZ = 10.0

# -------------------------------------------------------------- experiment
DEFAULT_SEED = 20261111
SCENES = ("hall", "warehouse", "obstacle")
