"""Constants for the musculoskeletal arm project (MSK-Arm)."""

from __future__ import annotations

import os

ROOT = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
MODELS_DIR = os.path.join(ROOT, "models")
RESULTS_DIR = os.path.join(ROOT, "results")
DOCS_DIR = os.path.join(ROOT, "docs")
ASSETS_DIR = os.path.join(DOCS_DIR, "assets")
MEDIA_DIR = os.path.join(ROOT, "media")

# ------------------------------------------------------------------ workspace
MAP_RESOLUTION = 0.010        # m / cell, workspace occupancy map
PLAN_RESOLUTION = 0.020
EE_RADIUS = 0.030             # end-effector radius (m)
INFLATION_RADIUS = 0.050
WORK_X = (-0.05, 0.64)        # workspace bounds (x forward)
WORK_Y = (-0.05, 0.64)        # workspace bounds (y up)
TARGET_TOLERANCE = 0.050      # m
HOLD_TIME = 0.30              # s inside tolerance to count a reach

# ------------------------------------------------------------- arm skeleton
L1 = 0.30                     # m, upper arm (shoulder -> elbow)
L2 = 0.28                     # m, forearm (elbow -> wrist)
M1 = 0.55                     # kg, upper arm (simplified, child-size)
M2 = 0.35                     # kg, forearm
I1 = 0.0042                   # kg m^2
I2 = 0.0022
DAMP1 = 0.90
DAMP2 = 0.50
Q1_MIN, Q1_MAX = -1.20, 1.60
Q2_MIN, Q2_MAX = 0.00, 2.60
GRAVITY = 9.81

# ----------------------------------------------------------------- muscles
# name, Fmax (N), optimal length (m), moment arm (m), flexor sign
MUSCLES = (
    ("shoulder_flexor", 260.0, 0.190, 0.024, +1.0),
    ("shoulder_extensor", 230.0, 0.190, 0.024, -1.0),
    ("elbow_flexor", 180.0, 0.160, 0.020, +1.0),
    ("elbow_extensor", 160.0, 0.160, 0.020, -1.0),
)
MUSCLE_TAU = 0.060            # s, activation dynamics
MUSCLE_FV = 0.25              # velocity factor (1 m/s of fibre speed)
CO_CONTRACTION_KEYS = True

# ------------------------------------------------------------------- sensors
CAM_WIDTH = 32
CAM_HEIGHT = 32
CAM_FOV = 2.0944              # 120 deg (arm-plane depth camera)
CAM_RANGE = 0.90
Q_SIGMA = 0.004               # rad
A_SIGMA = 0.010
FORCE_SIGMA = 0.05

# ------------------------------------------------------------------- control
CONTROL_HZ = 50.0
PHYSICS_HZ = 500.0
PHYS_DT = 1.0 / PHYSICS_HZ
SENSOR_HZ = 20.0

# -------------------------------------------------------------- experiment
DEFAULT_SEED = 20261111
SCENES = ("reach", "shelf", "draw")
# ------------------------------------------------------------------ aliases
ROBOT_RADIUS = EE_RADIUS
GOAL_TOLERANCE = TARGET_TOLERANCE
LIDAR_BEAMS = 72
LIDAR_RANGE_MAX = 0.90