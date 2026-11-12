#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""pi0-compatible policy interface (CPU stand-in).

In the reference setup, a pi0 VLA model consumes camera images + robot state +
a language instruction and outputs an action chunk, which is executed by the
MuJoCo environment.  Running pi0 needs a GPU (and the openpi weights), which is
not available in the target VM, so this module provides the same *interface*
with a scripted IK planner as the CPU fallback:

    obs = {"front": image, "wrist": image, "qpos": joint positions}
    actions = policy.act(obs)        # (chunk_size, action_dim)

The planner below produces the action plan once per episode; the demo executes
it through the same joint-position servo used by a real policy rollout.  To use
a real pi0 server, replace ``plan`` with the openpi client call (see README).
"""
import numpy as np

# scene constants (must match models/franka_emika_panda/scene_pickplace.xml)
CUBE_START = np.array([0.45, -0.12, 0.022])
PLATE_XY = np.array([0.45, 0.12])
TOP_DOWN_R = np.array([[1.0, 0.0, 0.0],
                       [0.0, -1.0, 0.0],
                       [0.0, 0.0, -1.0]])


class Pi0PolicyStub:
    """Same observation -> action-chunk semantics as pi0, CPU fallback planner."""

    name = "pi0-cpu-stub"
    action_dim = 8          # 7 joint position targets + 1 gripper command
    chunk_size = 8          # action chunk length (like pi0)
    control_hz = 30.0

    def describe(self):
        return ("pi0-cpu-stub: 7-DoF joint targets + gripper (chunk=%d, %d Hz). "
                "Real pi0 inference requires a GPU and openpi weights."
                % (self.chunk_size, int(self.control_hz)))

    def plan(self, env):
        """Return joint-space waypoints for the pick-and-place episode.

        Args:
            env: PandaPickPlace environment (exposes ik() and data.qpos).
        Returns:
            list of (name, q, ik_pos_error).
        """
        q_home = env.data.qpos[:7].copy()
        z_lift = 0.23
        targets = [
            ("pregrasp", np.array([CUBE_START[0], CUBE_START[1], z_lift]), TOP_DOWN_R),
            ("grasp",    np.array([CUBE_START[0], CUBE_START[1], 0.030]), TOP_DOWN_R),
            ("lift",     np.array([CUBE_START[0], CUBE_START[1], z_lift]), TOP_DOWN_R),
            ("above_plate", np.array([PLATE_XY[0], PLATE_XY[1], z_lift]), TOP_DOWN_R),
            ("place",    np.array([PLATE_XY[0], PLATE_XY[1], 0.070]), TOP_DOWN_R),
            ("retreat",  np.array([PLATE_XY[0], PLATE_XY[1], z_lift]), TOP_DOWN_R),
        ]
        waypoints = [("home", q_home, None)]
        q = q_home
        for name, pos, R in targets:
            q, err = env.ik(pos, R, q0=q)
            waypoints.append((name, q, np.linalg.norm(err[:3])))
            print("IK %-12s pos_err=%.5f rot_err=%.5f"
                  % (name, np.linalg.norm(err[:3]), np.linalg.norm(err[3:])))
        return waypoints
