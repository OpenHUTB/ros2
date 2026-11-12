# flake8: noqa  (vendored from lac-code, MIT; 保持上游原样，不参与本仓库 lint)
"""Per-joint constants for LAC deployment on the Unitree G1 (23 DOF).

All arrays of length 23 are in **JOINT_NAMES_23DOF order** (the order the policy
operates in, matching the training-time action-term joint ordering).

The SDK side uses a 27-slot layout (29DOF-truncated for 23DOF G1) with the
following empty slots: 13 (waist_roll), 14 (waist_pitch), 20 (left_wrist_pitch),
21 (left_wrist_yaw). Use ``JOINT_23_TO_SDK`` to translate from JOINT_NAMES_23DOF
index to SDK slot for the LowCmd / LowState messages (which have 35 slots; the
top 27 are populated, the rest stay at 0).

Values are copied verbatim from the training run's deploy parameters so the
deployment stack is self-contained.
"""

from __future__ import annotations

# ---- Joint name list (JOINT_NAMES_23DOF order) ----
JOINT_NAMES_23DOF: list[str] = [
    "left_hip_pitch_joint",     # 0
    "left_hip_roll_joint",      # 1
    "left_hip_yaw_joint",       # 2
    "left_knee_joint",          # 3
    "left_ankle_pitch_joint",   # 4
    "left_ankle_roll_joint",    # 5
    "right_hip_pitch_joint",    # 6
    "right_hip_roll_joint",     # 7
    "right_hip_yaw_joint",      # 8
    "right_knee_joint",         # 9
    "right_ankle_pitch_joint",  # 10
    "right_ankle_roll_joint",   # 11
    "waist_yaw_joint",          # 12
    "left_shoulder_pitch_joint",  # 13
    "left_shoulder_roll_joint",   # 14
    "left_shoulder_yaw_joint",    # 15
    "left_elbow_joint",           # 16
    "left_wrist_roll_joint",      # 17
    "right_shoulder_pitch_joint", # 18
    "right_shoulder_roll_joint",  # 19
    "right_shoulder_yaw_joint",   # 20
    "right_elbow_joint",          # 21
    "right_wrist_roll_joint",     # 22
]
NUM_JOINTS = 23

# ---- Per-joint action scale (from deploy.yaml actions.JointPositionAction.scale) ----
# q_target_i = action_i * scale_i + default_joint_pos_i
ACTION_SCALE_23: list[float] = [
    0.5475464652142303,  # 0 left_hip_pitch
    0.3506614663788243,  # 1 left_hip_roll
    0.5475464652142303,  # 2 left_hip_yaw
    0.3506614663788243,  # 3 left_knee
    0.4385773139233672,  # 4 left_ankle_pitch
    0.4385773139233672,  # 5 left_ankle_roll
    0.5475464652142303,  # 6 right_hip_pitch
    0.3506614663788243,  # 7 right_hip_roll
    0.5475464652142303,  # 8 right_hip_yaw
    0.3506614663788243,  # 9 right_knee
    0.4385773139233672,  # 10 right_ankle_pitch
    0.4385773139233672,  # 11 right_ankle_roll
    0.5475464652142303,  # 12 waist_yaw
    0.4385773139233672,  # 13 left_shoulder_pitch
    0.4385773139233672,  # 14 left_shoulder_roll
    0.4385773139233672,  # 15 left_shoulder_yaw
    0.4385773139233672,  # 16 left_elbow
    0.4385773139233672,  # 17 left_wrist_roll
    0.4385773139233672,  # 18 right_shoulder_pitch
    0.4385773139233672,  # 19 right_shoulder_roll
    0.4385773139233672,  # 20 right_shoulder_yaw
    0.4385773139233672,  # 21 right_elbow
    0.4385773139233672,  # 22 right_wrist_roll
]

# ---- Default joint position offset (from deploy.yaml actions.JointPositionAction.offset) ----
# q_target_i = action_i * scale_i + DEFAULT_JOINT_POS_23[i]
# Same array drives joint_pos_rel = dof_pos - DEFAULT_JOINT_POS_23 in obs_builder.
DEFAULT_JOINT_POS_23: list[float] = [
    -0.312,  # 0 left_hip_pitch
    0.0,     # 1 left_hip_roll
    0.0,     # 2 left_hip_yaw
    0.669,   # 3 left_knee
    -0.363,  # 4 left_ankle_pitch
    0.0,     # 5 left_ankle_roll
    -0.312,  # 6 right_hip_pitch
    0.0,     # 7 right_hip_roll
    0.0,     # 8 right_hip_yaw
    0.669,   # 9 right_knee
    -0.363,  # 10 right_ankle_pitch
    0.0,     # 11 right_ankle_roll
    0.0,     # 12 waist_yaw
    0.2,     # 13 left_shoulder_pitch
    0.2,     # 14 left_shoulder_roll
    0.0,     # 15 left_shoulder_yaw
    0.6,     # 16 left_elbow
    0.0,     # 17 left_wrist_roll
    0.2,     # 18 right_shoulder_pitch
    -0.2,    # 19 right_shoulder_roll
    0.0,     # 20 right_shoulder_yaw
    0.6,     # 21 right_elbow
    0.0,     # 22 right_wrist_roll
]

# ---- JOINT_NAMES_23DOF index -> SDK 27-slot index ----
# Legs and waist are 1:1 (SDK 0..12 maps directly).
# Arms have gaps: SDK 13/14 are waist_roll/pitch (unused in 23DOF), SDK 20/21
# are left_wrist_pitch/yaw (unused in 23DOF). Right arm starts at SDK 22.
JOINT_23_TO_SDK: list[int] = [
    0,   # 0 left_hip_pitch        -> SDK 0
    1,   # 1 left_hip_roll         -> SDK 1
    2,   # 2 left_hip_yaw          -> SDK 2
    3,   # 3 left_knee             -> SDK 3
    4,   # 4 left_ankle_pitch      -> SDK 4
    5,   # 5 left_ankle_roll       -> SDK 5
    6,   # 6 right_hip_pitch       -> SDK 6
    7,   # 7 right_hip_roll        -> SDK 7
    8,   # 8 right_hip_yaw         -> SDK 8
    9,   # 9 right_knee            -> SDK 9
    10,  # 10 right_ankle_pitch    -> SDK 10
    11,  # 11 right_ankle_roll     -> SDK 11
    12,  # 12 waist_yaw            -> SDK 12
    15,  # 13 left_shoulder_pitch  -> SDK 15 (skip 13/14 = waist_roll/pitch)
    16,  # 14 left_shoulder_roll   -> SDK 16
    17,  # 15 left_shoulder_yaw    -> SDK 17
    18,  # 16 left_elbow           -> SDK 18
    19,  # 17 left_wrist_roll      -> SDK 19
    22,  # 18 right_shoulder_pitch -> SDK 22 (skip 20/21 = left_wrist_pitch/yaw)
    23,  # 19 right_shoulder_roll  -> SDK 23
    24,  # 20 right_shoulder_yaw   -> SDK 24
    25,  # 21 right_elbow          -> SDK 25
    26,  # 22 right_wrist_roll     -> SDK 26
]

# SDK slots that have a 23DOF joint behind them (= JOINT_23_TO_SDK as a set, sorted).
# Used by damping/neutral-pose logic to decide which motor_cmd slots are "live".
SDK_SLOTS_ACTIVE_23DOF: list[int] = sorted(set(JOINT_23_TO_SDK))

# ---- Per-joint PD gains in JOINT_NAMES_23DOF order ----
# Derived from deploy.yaml top-level stiffness/damping (which are 27-long SDK order
# with 0 at slots 13/14/20/21) by indexing with JOINT_23_TO_SDK.
# Used by inference at POLICY/RAMP_UP states; KP_DAMPING/KP_NEUTRAL are separate (in inference.py).
STIFFNESS_23: list[float] = [
    40.18, 99.10, 40.18, 99.10, 28.50, 28.50,  # left leg (hip_p/hip_r/hip_y/knee/ank_p/ank_r)
    40.18, 99.10, 40.18, 99.10, 28.50, 28.50,  # right leg
    40.18,                                     # waist_yaw
    14.30, 14.30, 14.30, 14.30, 14.30,         # left arm (shoulder_p/r/y, elbow, wrist_r)
    14.30, 14.30, 14.30, 14.30, 14.30,         # right arm
]
DAMPING_23: list[float] = [
    2.56, 6.31, 2.56, 6.31, 1.81, 1.81,
    2.56, 6.31, 2.56, 6.31, 1.81, 1.81,
    2.56,
    0.907, 0.907, 0.907, 0.907, 0.907,
    0.907, 0.907, 0.907, 0.907, 0.907,
]

# ---- Articulation order (IsaacLab internal sorted order) ----
# CRITICAL for obs construction: ``isaaclab.envs.mdp.observations.joint_pos_rel``
# returns the FULL joint_pos vector via ``asset.data.joint_pos[:, slice(None)]``
# which is in Articulation order, NOT JOINT_NAMES_23DOF order. So obs terms
# ``joint_pos_rel`` and ``joint_vel_rel`` must be built in Articulation order
# at deploy time. ``last_action`` stays in JOINT_NAMES_23DOF order because the
# action manager preserves the policy's input order.
#
# Articulation order for G1 23DOF (IsaacLab's internal breadth-first joint sort):
#   ART 0:  left_hip_pitch      ART 12: right_knee
#   ART 1:  right_hip_pitch     ART 13: left_shoulder_yaw
#   ART 2:  waist_yaw           ART 14: right_shoulder_yaw
#   ART 3:  left_hip_roll       ART 15: left_ankle_pitch
#   ART 4:  right_hip_roll      ART 16: right_ankle_pitch
#   ART 5:  left_shoulder_pitch ART 17: left_elbow
#   ART 6:  right_shoulder_pitch ART 18: right_elbow
#   ART 7:  left_hip_yaw        ART 19: left_ankle_roll
#   ART 8:  right_hip_yaw       ART 20: right_ankle_roll
#   ART 9:  left_shoulder_roll  ART 21: left_wrist_roll
#   ART 10: right_shoulder_roll ART 22: right_wrist_roll
#   ART 11: left_knee

# JOINT_NAMES_23DOF index -> Articulation index (matches deploy.yaml actions.JointPositionAction.joint_ids)
JOINT_23_TO_ART: list[int] = [
    0, 3, 7, 11, 15, 19,    # left leg (hip_p/hip_r/hip_y/knee/ank_p/ank_r)
    1, 4, 8, 12, 16, 20,    # right leg
    2,                      # waist_yaw
    5, 9, 13, 17, 21,       # left arm (shoulder_p/r/y, elbow, wrist_r)
    6, 10, 14, 18, 22,      # right arm
]

# Articulation index -> JOINT_NAMES_23DOF index (inverse permutation; used to
# convert dof_pos read in JOINT_23 order back to ART order for obs construction).
ART_TO_J23: list[int] = [0] * NUM_JOINTS
for _i, _art in enumerate(JOINT_23_TO_ART):
    ART_TO_J23[_art] = _i

# Default joint position in Articulation order (from deploy.yaml top-level
# default_joint_pos field). Used as the offset in joint_pos_rel = dof_pos - default
# at obs-construction time.
DEFAULT_JOINT_POS_ART: list[float] = [
    -0.312,  # 0  left_hip_pitch
    -0.312,  # 1  right_hip_pitch
    0.0,     # 2  waist_yaw
    0.0,     # 3  left_hip_roll
    0.0,     # 4  right_hip_roll
    0.2,     # 5  left_shoulder_pitch
    0.2,     # 6  right_shoulder_pitch
    0.0,     # 7  left_hip_yaw
    0.0,     # 8  right_hip_yaw
    0.2,     # 9  left_shoulder_roll
    -0.2,    # 10 right_shoulder_roll
    0.669,   # 11 left_knee
    0.669,   # 12 right_knee
    0.0,     # 13 left_shoulder_yaw
    0.0,     # 14 right_shoulder_yaw
    -0.363,  # 15 left_ankle_pitch
    -0.363,  # 16 right_ankle_pitch
    0.6,     # 17 left_elbow
    0.6,     # 18 right_elbow
    0.0,     # 19 left_ankle_roll
    0.0,     # 20 right_ankle_roll
    0.0,     # 21 left_wrist_roll
    0.0,     # 22 right_wrist_roll
]

# Sanity: DEFAULT_JOINT_POS_ART derived from DEFAULT_JOINT_POS_23 via inverse permutation
# (verifies our two arrays + permutation are mutually consistent).
_default_from_23 = [0.0] * NUM_JOINTS
for _i23, _art in enumerate(JOINT_23_TO_ART):
    _default_from_23[_art] = DEFAULT_JOINT_POS_23[_i23]
for _art in range(NUM_JOINTS):
    assert abs(_default_from_23[_art] - DEFAULT_JOINT_POS_ART[_art]) < 1e-6, (
        f"DEFAULT inconsistency at ART {_art}: from_23={_default_from_23[_art]} "
        f"vs hardcoded={DEFAULT_JOINT_POS_ART[_art]}"
    )

# ---- Sanity checks at import time ----
assert len(JOINT_NAMES_23DOF) == NUM_JOINTS
assert len(ACTION_SCALE_23) == NUM_JOINTS
assert len(DEFAULT_JOINT_POS_23) == NUM_JOINTS
assert len(JOINT_23_TO_SDK) == NUM_JOINTS
assert len(STIFFNESS_23) == NUM_JOINTS
assert len(DAMPING_23) == NUM_JOINTS
assert len(SDK_SLOTS_ACTIVE_23DOF) == NUM_JOINTS, "JOINT_23_TO_SDK has duplicates"
assert max(JOINT_23_TO_SDK) <= 26, f"SDK slot > 26: {max(JOINT_23_TO_SDK)}"
assert len(JOINT_23_TO_ART) == NUM_JOINTS
assert len(set(JOINT_23_TO_ART)) == NUM_JOINTS, "JOINT_23_TO_ART has duplicates"
assert sorted(JOINT_23_TO_ART) == list(range(NUM_JOINTS)), "JOINT_23_TO_ART not a permutation of 0..22"
assert len(DEFAULT_JOINT_POS_ART) == NUM_JOINTS
