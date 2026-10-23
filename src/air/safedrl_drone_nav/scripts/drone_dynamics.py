"""
Rigid-body dynamics, motor model and geometric inner-loop controller for a
CrazyFlie-2.X class quadrotor, implemented directly on top of PyBullet.

The parameter set intentionally mirrors the one used by the well known
``pybullet_drones`` package (``BaseAviary``) so the numbers quoted in the
accompanying report stay comparable with the reference literature:

    m       = 0.027 kg            arm      = 0.0397 m
    kf      = 3.16e-10 N/rpm^2    km       = 7.94e-12 N*m/rpm^2
    Ixx=Iyy = 1.4e-5 kg*m^2       Izz      = 2.17e-5 kg*m^2
    max_rpm = 25000 rpm

Control architecture (outer loop -> inner loop):

    RL action  : desired world-frame velocity v_des [m/s] + yaw-rate [rad/s]
    outer loop : PD  v_des -> desired net acceleration a_des
    attitude   : geometric SE(3) controller  a_des -> (thrust, body torques)
    mixer      : (thrust, torques) -> per-rotor rpm (exact pseudo-inverse)
    motors     : first-order lag on rpm (physical ESC/motor response)

The safety filter (``safety.py``) sits *between* the RL policy and the outer
loop, i.e. it filters ``v_des`` before it is ever handed to the controller.
This is what makes the hard safety guarantee meaningful: the barrier condition
is enforced on the signal that actually drives the vehicle.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import pybullet as p

# --------------------------------------------------------------------------
# Physical parameters
# --------------------------------------------------------------------------


def _wrap_pi(angle: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    return float((angle + np.pi) % (2.0 * np.pi) - np.pi)


@dataclass(frozen=True)
class CF2XParams:
    """CrazyFlie 2.X class quadrotor parameters."""

    mass: float = 0.027          # [kg]
    arm: float = 0.0397          # [m] motor offset along x and y
    kf: float = 3.16e-10         # [N / rpm^2]  thrust coefficient
    km: float = 7.94e-12         # [N*m / rpm^2] drag (yaw) coefficient
    ixx: float = 1.4e-5          # [kg*m^2]
    iyy: float = 1.4e-5          # [kg*m^2]
    izz: float = 2.17e-5         # [kg*m^2]
    max_rpm: float = 25000.0     # [rpm]
    min_rpm: float = 0.0         # [rpm]
    prop_radius: float = 0.023   # [m]
    drag_coeff: float = 6.1e-3   # [N/(m/s)^2]  quadratic body drag
    ang_damp: float = 6.0e-7     # [N*m/(rad/s)] linear angular damping
    motor_tau: float = 0.02      # [s] first-order motor/ESC lag
    gravity: float = 9.81        # [m/s^2]

    @property
    def hover_rpm(self) -> float:
        return float(np.sqrt(self.mass * self.gravity / (4.0 * self.kf)))

    @property
    def max_thrust(self) -> float:
        return 4.0 * self.kf * self.max_rpm ** 2


CF2X = CF2XParams()

# Motor layout (X configuration), metres, body frame.
#   index 0 : front-right    index 1 : rear-left
#   index 2 : front-left     index 3 : rear-right
PROP_POSITIONS = np.array(
    [
        [+CF2X.arm, +CF2X.arm, 0.0],
        [-CF2X.arm, -CF2X.arm, 0.0],
        [+CF2X.arm, -CF2X.arm, 0.0],
        [-CF2X.arm, +CF2X.arm, 0.0],
    ]
)
# Spin direction (+1 = counter-clockwise when looking down at the rotor)
PROP_DIRECTIONS = np.array([+1.0, +1.0, -1.0, -1.0])


def build_allocation_matrix(params: CF2XParams = CF2X) -> np.ndarray:
    """Return the 4x4 matrix mapping rotor thrusts to (T, tx, ty, tz).

    ``A @ [F0, F1, F2, F3] = [T, tau_x, tau_y, tau_z]``

    The yaw row is expressed through the thrust vector by using
    ``tau_z = (km / kf) * sum(dir_i * F_i)`` which keeps the linear system
    square and directly invertible.
    """
    a = params.arm
    ratio = params.km / params.kf
    row_t = np.ones(4)
    row_x = np.array([+a, -a, -a, +a]) * 1.0  # from cross(prop_pos, F z_hat)
    row_y = np.array([-a, -a, +a, +a]) * 1.0
    row_z = ratio * PROP_DIRECTIONS
    # Recompute rigorously from the geometry to avoid sign typos.
    row_x = np.array([+PROP_POSITIONS[i][1] for i in range(4)])
    row_y = np.array([-PROP_POSITIONS[i][0] for i in range(4)])
    return np.vstack([row_t, row_x, row_y, row_z])


class QuadrotorBody:
    """A PyBullet multi-body quadrotor with an explicit inertia tensor."""

    def __init__(
        self,
        client: int,
        params: CF2XParams = CF2X,
        start_pos=(0.0, 0.0, 0.0),
        start_quat=(0.0, 0.0, 0.0, 1.0),
        rgba=(0.19, 0.55, 0.91, 1.0),
    ):
        self.client = client
        self.p = params
        self.alloc = build_allocation_matrix(params)
        self.alloc_inv = np.linalg.inv(self.alloc)

        body_half = [0.030, 0.030, 0.008]
        col_base = p.createCollisionShape(
            p.GEOM_BOX, halfExtents=body_half, physicsClientId=client
        )

        # --- compound visual: body plate + four rotor discs -----------------
        shape_types, half_extents, radii, lengths, frames, colours = [], [], [], [], [], []
        shape_types.append(p.GEOM_BOX)
        half_extents.append(body_half)
        radii.append(0.0)
        lengths.append(0.0)
        frames.append([0, 0, 0])
        colours.append(list(rgba))

        for i, pos in enumerate(PROP_POSITIONS):
            shape_types.append(p.GEOM_CYLINDER)
            half_extents.append([0, 0, 0])
            radii.append(params.prop_radius)
            lengths.append(0.004)
            frames.append([float(pos[0]), float(pos[1]), 0.008])
            # counter-rotating pairs get slightly different tint
            colours.append([0.90, 0.30, 0.25, 0.85] if PROP_DIRECTIONS[i] > 0
                           else [0.25, 0.35, 0.90, 0.85])

        visual = p.createVisualShapeArray(
            shapeTypes=shape_types,
            halfExtents=half_extents,
            radii=radii,
            lengths=lengths,
            visualFramePositions=frames,
            rgbaColors=colours,
            physicsClientId=client,
        )

        self.body_id = p.createMultiBody(
            baseMass=params.mass,
            baseCollisionShapeIndex=col_base,
            baseVisualShapeIndex=visual,
            basePosition=list(start_pos),
            baseOrientation=list(start_quat),
            physicsClientId=client,
        )
        p.changeDynamics(
            self.body_id,
            -1,
            localInertiaDiagonal=[params.ixx, params.iyy, params.izz],
            linearDamping=0.0,
            angularDamping=0.0,
            restitution=0.15,
            lateralFriction=0.6,
            physicsClientId=client,
        )

        # motor state (rpm), integrated with a first-order lag
        self.rpm = np.full(4, params.hover_rpm, dtype=np.float64)

    # ------------------------------------------------------------------
    # state accessors
    # ------------------------------------------------------------------
    def state(self):
        """Return (pos, quat, euler, vel, omega) in world / body frames."""
        pos, quat = p.getBasePositionAndOrientation(self.body_id, physicsClientId=self.client)
        vel, omega = p.getBaseVelocity(self.body_id, physicsClientId=self.client)
        euler = p.getEulerFromQuaternion(quat)
        return (
            np.asarray(pos, dtype=np.float64),
            np.asarray(quat, dtype=np.float64),
            np.asarray(euler, dtype=np.float64),
            np.asarray(vel, dtype=np.float64),
            np.asarray(omega, dtype=np.float64),
        )

    def rotation_matrix(self) -> np.ndarray:
        _, quat, _, _, _ = self.state()
        return np.asarray(p.getMatrixFromQuaternion(quat), dtype=np.float64).reshape(3, 3)

    def reset(self, pos, quat=(0.0, 0.0, 0.0, 1.0), rpm=None):
        p.resetBasePositionAndOrientation(
            self.body_id, list(pos), list(quat), physicsClientId=self.client
        )
        p.resetBaseVelocity(
            self.body_id, [0, 0, 0], [0, 0, 0], physicsClientId=self.client
        )
        self.rpm = np.full(4, self.p.hover_rpm if rpm is None else rpm, dtype=np.float64)

    # ------------------------------------------------------------------
    # low level actuation
    # ------------------------------------------------------------------
    def apply_rpm(self, rpm_cmd: np.ndarray, dt: float, wind_world: np.ndarray | None = None):
        """Apply rotor speeds (with motor lag), quadratic drag and gravity."""
        rpm_cmd = np.clip(np.asarray(rpm_cmd, dtype=np.float64), self.p.min_rpm, self.p.max_rpm)
        alpha = 1.0 - np.exp(-dt / self.p.motor_tau)
        self.rpm = self.rpm + alpha * (rpm_cmd - self.rpm)

        forces = self.p.kf * self.rpm ** 2
        tau = self.alloc[1:] @ forces  # [tx, ty, tz]
        thrust = float(np.sum(forces))

        rot = self.rotation_matrix()
        _, _, _, vel, omega = self.state()
        omega = np.asarray(omega, dtype=np.float64)

        # Everything is applied in the LINK frame at the body origin, which is
        # the centre of mass.  This matters: passing WORLD_FRAME together with
        # posObj=[0,0,0] would apply the force at the *world origin*, creating
        # a spurious torque of magnitude |com x F| that grows with altitude and
        # destabilises the vehicle.
        if wind_world is None:
            wind_world = np.zeros(3)
        v_rel_body = rot.T @ (vel - np.asarray(wind_world, dtype=np.float64))

        # aerodynamic drag acts on the *air-relative* velocity -> wind enters here
        speed = float(np.linalg.norm(v_rel_body))
        drag_body = -self.p.drag_coeff * speed * v_rel_body
        force_body = np.array([0.0, 0.0, thrust]) + drag_body

        # `tau` comes out of the mixer already in body axes
        torque_body = tau - self.p.ang_damp * (rot.T @ omega)

        p.applyExternalForce(
            self.body_id, -1, forceObj=list(force_body),
            posObj=[0, 0, 0], flags=p.LINK_FRAME, physicsClientId=self.client,
        )
        p.applyExternalTorque(
            self.body_id, -1, torqueObj=list(torque_body),
            flags=p.LINK_FRAME, physicsClientId=self.client,
        )

    # ------------------------------------------------------------------
    # allocation
    # ------------------------------------------------------------------
    def thrust_torque_to_rpm(self, thrust: float, torque: np.ndarray) -> np.ndarray:
        """Invert the mixer: desired (T, tau) -> rotor rpm."""
        rhs = np.array([thrust, torque[0], torque[1], torque[2]], dtype=np.float64)
        forces = self.alloc_inv @ rhs
        forces = np.clip(forces, 0.0, self.p.kf * self.p.max_rpm ** 2)
        return np.sqrt(forces / self.p.kf)


# --------------------------------------------------------------------------
# Geometric SE(3) controller
# --------------------------------------------------------------------------


class GeometricController:
    """Cascaded velocity -> acceleration -> attitude -> rotor-speed controller.

    Velocity loop (world frame, proportional)::

        a_des = kp_v * (v_des - v)                     [clipped to +-a_max]

    Because ``v_dot = a_des`` the closed loop is first order with pole ``kp_v``,
    i.e. the velocity converges to the command with time constant ``1/kp_v``.

    Attitude loop (Lee et al. geometric tracking control on SO(3))::

        e_R   = 0.5 * vee(R_des^T R - R^T R_des)
        e_w   = omega - R^T R_des omega_des
        tau   = -kp_R * e_R - kd_w * e_w + omega x (I omega)

    Gains are *derived from the airframe inertia* rather than guessed:

        kp_R = I * wn^2            (wn ~ 18 rad/s -> ~3 s^-1 bandwidth)
        kd_w = 2 * zeta * I * wn   (zeta ~ 1.0, critically damped)

    For ``I = 1.4e-5 kg m^2`` this gives ``kp_R = 4.5e-3``, ``kd_w = 5.1e-4``.
    The available differential torque is ~1.6e-2 N*m, so a 0.1 rad attitude
    error demands 4.5e-4 N*m - a 35x margin against saturation.  (Using gains
    an order of magnitude above this makes the explicit 240 Hz integration
    diverge, which is the classic pybullet-drones tuning trap.)
    """

    def __init__(
        self,
        params: CF2XParams = CF2X,
        kp_v: float = 5.0,
        a_max: float = 7.5,
        wn_att: float = 18.0,
        zeta_att: float = 1.0,
        att_scale: float = 1.0,
        max_yaw_rate: float = 2.5,
    ):
        self.p = params
        self.kp_v = kp_v
        self.a_max = a_max
        self.max_yaw_rate = max_yaw_rate
        self.inertia = np.diag([params.ixx, params.iyy, params.izz])
        self.kp_R = att_scale * params.ixx * wn_att ** 2
        self.kd_w = att_scale * 2.0 * zeta_att * params.ixx * wn_att
        self._yaw_ref = 0.0

    def reset(self, yaw: float = 0.0):
        self._yaw_ref = float(yaw)

    def compute(
        self,
        body: QuadrotorBody,
        v_des: np.ndarray,
        yaw_rate_des: float,
        dt: float,
    ):
        """Return the rotor rpm command realising ``v_des`` / ``yaw_rate_des``."""
        _, _, _, vel, omega = body.state()
        rot = body.rotation_matrix()

        yaw_rate_des = float(np.clip(yaw_rate_des, -self.max_yaw_rate, self.max_yaw_rate))
        self._yaw_ref = _wrap_pi(self._yaw_ref + yaw_rate_des * dt)
        psi = self._yaw_ref

        # ---- velocity loop -------------------------------------------------
        v_des = np.asarray(v_des, dtype=np.float64)
        a_des = self.kp_v * (v_des - np.asarray(vel, dtype=np.float64))
        a_des = np.clip(a_des, -self.a_max, self.a_max)
        f_des = self.p.mass * (a_des + np.array([0.0, 0.0, self.p.gravity]))

        # ---- desired attitude ---------------------------------------------
        b3_des = f_des / max(np.linalg.norm(f_des), 1e-9)
        b1_des = np.array([np.cos(psi), np.sin(psi), 0.0])
        b2_des = np.cross(b3_des, b1_des)
        n2 = np.linalg.norm(b2_des)
        if n2 < 1e-6:  # thrust vector parallel to the heading -> pick any
            b1_des = np.array([1.0, 0.0, 0.0])
            b2_des = np.cross(b3_des, b1_des)
            n2 = np.linalg.norm(b2_des)
        b2_des = b2_des / n2
        b1_des = np.cross(b2_des, b3_des)
        r_des = np.column_stack([b1_des, b2_des, b3_des])

        # ---- attitude error on SO(3) --------------------------------------
        r_err = r_des.T @ rot - rot.T @ r_des
        e_r = 0.5 * np.array([r_err[2, 1], r_err[0, 2], r_err[1, 0]])
        omega_des = np.array([0.0, 0.0, yaw_rate_des])
        e_w = np.asarray(omega, dtype=np.float64) - rot.T @ r_des @ omega_des

        tau = (
            -self.kp_R * e_r
            - self.kd_w * e_w
            + np.cross(np.asarray(omega, dtype=np.float64), self.inertia @ np.asarray(omega, dtype=np.float64))
        )

        thrust = float(np.dot(f_des, rot[:, 2]))
        thrust = float(np.clip(thrust, 0.0, self.p.max_thrust))

        return body.thrust_torque_to_rpm(thrust, tau)
