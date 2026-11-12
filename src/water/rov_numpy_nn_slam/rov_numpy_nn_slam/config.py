"""Shared constants and paths for the RVNN ROV project."""

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
MAP_RESOLUTION = 0.05        # m / cell, occupancy grid used by SLAM
PLAN_RESOLUTION = 0.10       # m / cell, coarse grid used by the A* planner
ROV_RADIUS = 0.42            # collision / inflation radius (m)
INFLATION_RADIUS = 0.55

# ------------------------------------------------------------------- vehicle
HULL_HALF = (0.32, 0.26, 0.16)   # MuJoCo box half extents (m)
MASS = 12.0                      # kg
WATER_DENSITY = 1025.0           # kg / m^3
GRAVITY = 9.81
VOLUME = 0.01195                 # m^3, gives ~ +1 N net buoyancy
MAX_SURGE = 1.20                 # m/s
MAX_SWAY = 0.80
MAX_HEAVE = 0.60
MAX_YAW_RATE = 1.20              # rad/s
MAX_DEPTH = 3.40                 # m below surface
MIN_DEPTH = 0.35

# ------------------------------------------------------------------- sensors
SONAR_BEAMS = 64
SONAR_FOV = 2.0944               # 120 deg horizontal fan
SONAR_RANGE_MIN = 0.30
SONAR_RANGE_MAX = 8.00
CAM_WIDTH = 64
CAM_HEIGHT = 48
CAM_FOV = 1.5708                 # 90 deg
CAM_RANGE = 6.0

# ------------------------------------------------------------------- control
CONTROL_HZ = 50.0
PHYSICS_HZ = 200.0
PHYS_DT = 1.0 / PHYSICS_HZ
SENSOR_HZ = 10.0

# -------------------------------------------------------------- experiment
DEFAULT_SEED = 20261110
SCENES = ("pool", "harbor", "cave")
