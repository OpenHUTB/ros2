"""Shared paths and constants."""

from __future__ import annotations

import os

ROOT = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))

MODELS_DIR = os.path.join(ROOT, "models")
RESULTS_DIR = os.path.join(ROOT, "results")
DOCS_DIR = os.path.join(ROOT, "docs")
ASSETS_DIR = os.path.join(DOCS_DIR, "assets")
CONFIG_DIR = os.path.join(ROOT, "config")

# World / map
MAP_RESOLUTION = 0.05          # metres per occupancy-grid cell
PLAN_RESOLUTION = 0.10         # coarser grid used by A*
ROBOT_RADIUS = 0.36            # collision radius used for inflation
INFLATION_RADIUS = 0.42

# Vehicle geometry (must match the MuJoCo model)
WHEEL_RADIUS = 0.10
WHEEL_TRACK = 0.44
CHASSIS_Z = 0.13

# Sensor model
LIDAR_BEAMS = 360
LIDAR_RANGE_MAX = 9.0
LIDAR_RANGE_MIN = 0.08
LIDAR_HEIGHT = 0.18
CAMERA_SIZE = 64
BEV_SIZE = 64
BEV_EXTENT = 12.0              # metres covered by the bird's-eye camera

# Control
CONTROL_HZ = 50.0
PHYSICS_HZ = 500.0
SENSOR_HZ = 10.0
MAX_SPEED = 1.10
MAX_OMEGA = 1.60

# Experiment defaults
DEFAULT_SEED = 20261026
