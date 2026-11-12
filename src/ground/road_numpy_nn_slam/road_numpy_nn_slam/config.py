"""Shared constants and paths for the road-vehicle (GVNN-Road) project."""

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
MAP_RESOLUTION = 0.10        # m / cell, occupancy grid used by SLAM
PLAN_RESOLUTION = 0.25       # m / cell, coarse grid used by the A* planner
CAR_RADIUS = 1.05            # vehicle safety radius (m)
INFLATION_RADIUS = 1.55

# --------------------------------------------------------------- vehicle model
WHEELBASE = 2.60             # m
MAX_SPEED = 12.0             # m/s (~43 km/h)
MAX_REVERSE = 3.0
MAX_ACCEL = 3.0              # m/s^2
MAX_BRAKE = 6.0
DRAG = 0.020                 # 1/m, quadratic air drag
ROLLING = 0.18               # m/s^2, rolling resistance
MAX_STEER = 0.60             # rad
STEER_RATE = 1.60            # rad/s
UNDERSTEER = 0.0025          # s^2/m^2, understeer gradient
OFFROAD_DRAG = 1.5           # extra deceleration off the road

# ------------------------------------------------------------------- sensors
LIDAR_BEAMS = 72
LIDAR_FOV = 6.2831853        # 360 deg
LIDAR_RANGE_MIN = 0.30
LIDAR_RANGE_MAX = 40.0
CAM_WIDTH = 64
CAM_HEIGHT = 48
CAM_FOV = 1.0472             # 60 deg forward camera
CAM_RANGE = 40.0
GNSS_SIGMA = 0.35            # m
IMU_SIGMA = 0.002

# ------------------------------------------------------------------- control
CONTROL_HZ = 20.0
PHYSICS_HZ = 100.0
PHYS_DT = 1.0 / PHYSICS_HZ
SENSOR_HZ = 10.0

# --------------------------------------------------------------- traffic light
LIGHT_CYCLE = 17.0           # s: green 8 + yellow 2 + red 7
LIGHT_GREEN = 8.0
LIGHT_YELLOW = 2.0

# -------------------------------------------------------------- experiment
DEFAULT_SEED = 20261110
SCENES = ("downtown", "campus", "construction")

# ------------------------------------------------------------------ aliases
# Backwards-compatible names for code shared with the other vehicle samples.
SONAR_BEAMS = LIDAR_BEAMS
SONAR_FOV = LIDAR_FOV
SONAR_RANGE_MIN = LIDAR_RANGE_MIN
SONAR_RANGE_MAX = LIDAR_RANGE_MAX
MAX_SURGE = MAX_SPEED
MAX_SWAY = 0.0
MAX_HEAVE = 0.0
MAX_OMEGA = 1.2
MAX_DEPTH = 0.0
MIN_DEPTH = 0.0
DEFAULT_ALT = 0.0