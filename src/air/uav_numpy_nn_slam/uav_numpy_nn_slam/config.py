"""Shared constants and paths for the UAVNN aerial-vehicle project."""

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
MAP_RESOLUTION = 0.20        # m / cell, occupancy grid used by SLAM
PLAN_RESOLUTION = 0.40       # m / cell, coarse grid used by the A* planner
UAV_RADIUS = 1.20            # safety radius (m)
INFLATION_RADIUS = 1.80

# ------------------------------------------------------------------- vehicle
BODY_HALF = (0.22, 0.22, 0.06)   # MuJoCo box half extents (m)
MASS = 1.40                      # kg
ARM = 0.22                       # rotor arm length (m)
GRAVITY = 9.81
HOVER_THRUST = MASS * GRAVITY    # N
THRUST_MAX = 26.0                # N total (4 rotors)
MAX_VX = 8.0                     # m/s
MAX_VY = 5.0
MAX_VZ = 3.0
MAX_YAW_RATE = 1.20              # rad/s
DEFAULT_ALT = 30.0               # m
MIN_ALT = 8.0
MAX_ALT = 55.0

# ------------------------------------------------------------------- sensors
LIDAR_BEAMS = 72
LIDAR_FOV = 6.2831853            # 360 deg
LIDAR_RANGE_MIN = 0.50
LIDAR_RANGE_MAX = 60.0
CAM_WIDTH = 64
CAM_HEIGHT = 48
CAM_FOV = 1.5708                 # 90 deg forward camera
CAM_RANGE = 50.0
BARO_SIGMA = 0.06                # m
GPS_SIGMA = 0.45                 # m

# ------------------------------------------------------------------- control
CONTROL_HZ = 20.0
PHYSICS_HZ = 200.0
PHYS_DT = 1.0 / PHYSICS_HZ
SENSOR_HZ = 10.0

# --------------------------------------------------------------------- wind
WIND_BASE = (1.6, 0.7)           # m/s mean wind (x, y)
WIND_GUST = 1.2                  # m/s gust amplitude
WIND_TAU = 6.0                   # s, gust correlation time

# -------------------------------------------------------------- experiment
DEFAULT_SEED = 20261110
SCENES = ("city", "canyon", "airport")

# ------------------------------------------------------------------ aliases
# Backwards-compatible names so that modules written for the previous
# (underwater) sample keep working after the aerial retarget.
SONAR_BEAMS = LIDAR_BEAMS
SONAR_FOV = LIDAR_FOV
SONAR_RANGE_MIN = LIDAR_RANGE_MIN
SONAR_RANGE_MAX = LIDAR_RANGE_MAX
MAX_SURGE = MAX_VX
MAX_SWAY = MAX_VY
MAX_HEAVE = MAX_VZ
MAX_OMEGA = MAX_YAW_RATE
MAX_DEPTH = MAX_ALT
MIN_DEPTH = MIN_ALT