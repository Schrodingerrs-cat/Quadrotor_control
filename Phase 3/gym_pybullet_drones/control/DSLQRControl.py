import math

import numpy as np
import pybullet as p
from scipy.linalg import solve_continuous_are
from scipy.spatial.transform import Rotation

from gym_pybullet_drones.control.BaseControl import BaseControl
from gym_pybullet_drones.utils.enums import DroneModel


class DSLQRControl(BaseControl):
    """Full-state LQR around hover (bonus part).

    State:  x = [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r]^T
    Input:  u = [u1, tau_phi, tau_theta, tau_psi]^T
    Linearized at hover: (r = 0, phi = theta = 0, psi = psi0, u1 = mg, tau = 0)
    """

    def __init__(self, drone_model: DroneModel, g: float = 9.8):
        super().__init__(drone_model=drone_model, g=g)

        if self.DRONE_MODEL not in [DroneModel.CF2X, DroneModel.CF2P]:
            print("[ERROR] DSLQRControl requires Crazyflie 2.X models")
            exit()

        # Physical parameters from URDF
        self.mass = float(self._getURDFParameter("m"))
        self.Ixx = float(self._getURDFParameter("ixx"))
        self.Iyy = float(self._getURDFParameter("iyy"))
        self.Izz = float(self._getURDFParameter("izz"))
        self.I = np.diag([self.Ixx, self.Iyy, self.Izz])

        # BaseControl.GRAVITY is m * g (force), convert to linear acceleration
        self.g_lin = self.GRAVITY / self.mass  # ≈ 9.8 m/s^2

        # Rotor / PWM conversion
        self.PWM2RPM_SCALE = 0.2685
        self.PWM2RPM_CONST = 4070.3
        self.MIN_PWM = 20000
        self.MAX_PWM = 65535

        # Mixer matrix (same as DSLPIDControl)
        if self.DRONE_MODEL == DroneModel.CF2X:
            self.MIXER_MATRIX = np.array(
                [[-0.5, -0.5, -1], [-0.5, 0.5, 1], [0.5, 0.5, -1], [0.5, -0.5, 1]]
            )
        else:  # CF2P
            self.MIXER_MATRIX = np.array(
                [[0, -1, -1], [1, 0, 1], [0, 1, -1], [-1, 0, 1]]
            )

        # =========================================================
        #  Linearized model at hover (bonus derivation)
        #  x = [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r]
        # =========================================================
        m = self.mass
        Ix, Iy, Iz = self.Ixx, self.Iyy, self.Izz
        g = self.g_lin

        A = np.zeros((12, 12))

        # Position derivatives: xdot = vx, ydot = vy, zdot = vz
        A[0, 3] = 1.0
        A[1, 4] = 1.0
        A[2, 5] = 1.0

        # Linearized translational dynamics:
        # vx_dot = g * theta
        # vy_dot = -g * phi
        A[3, 7] = g  # vx_dot depends on theta
        A[4, 6] = -g  # vy_dot depends on phi

        # z dynamics are driven by delta u1 (through B), not A

        # Euler angle kinematics (small angles):
        # phi_dot = p, theta_dot = q, psi_dot = r
        A[6, 9] = 1.0
        A[7, 10] = 1.0
        A[8, 11] = 1.0

        # Angular rates dynamics are driven by torques (through B)

        B = np.zeros((12, 4))

        # z-acceleration: z_ddot = delta_u1 / m
        B[5, 0] = 1.0 / m

        # p_dot = tau_phi / Ix, q_dot = tau_theta / Iy, r_dot = tau_psi / Iz
        B[9, 1] = 1.0 / Ix
        B[10, 2] = 1.0 / Iy
        B[11, 3] = 1.0 / Iz

        # =========================================================
        #                   LQR WEIGHT MATRICES
        #  These are tuned for a local hover regulator (not large
        #  point-to-point maneuvers).
        # =========================================================
        self.Q = np.diag(
            [
                5.0,
                5.0,
                8.0,  # x, y, z
                1.0,
                1.0,
                2.0,  # vx, vy, vz
                20.0,
                20.0,
                10.0,  # phi, theta, psi
                2.0,
                2.0,
                1.0,  # p, q, r
            ]
        )

        # Penalize large thrust and torques strongly to avoid saturation
        self.R = np.diag(
            [
                10.0,  # u1
                0.5,  # tau_phi
                0.5,  # tau_theta
                0.5,  # tau_psi
            ]
        )

        # Solve continuous-time Riccati equation and get gain K
        P = solve_continuous_are(A, B, self.Q, self.R)
        self.K = np.linalg.inv(self.R) @ (B.T @ P)

        # Hover total thrust (Newton)
        self.hover_thrust = m * g

        self.reset()

    # --------------------------------------------------------------
    def reset(self):
        super().reset()

    # --------------------------------------------------------------
    def computeControl(
        self,
        control_timestep,
        cur_pos,
        cur_quat,
        cur_vel,
        cur_ang_vel,
        target_pos,
        target_rpy=np.zeros(3),
        target_vel=np.zeros(3),
        target_rpy_rates=np.zeros(3),
        target_acc=np.zeros(3),
    ):
        """
        Full-state LQR. This is a LOCAL hover regulator:
        it assumes the drone stays near small angles and small velocities.
        """

        # Current rotation and Euler angles
        R_wb = np.array(p.getMatrixFromQuaternion(cur_quat)).reshape(3, 3)
        euler = Rotation.from_matrix(R_wb).as_euler("xyz", degrees=False)
        phi, theta, psi = euler

        # ------------------------------------------------------------------
        #              Build current 12D state vector
        # ------------------------------------------------------------------
        state = np.array(
            [
                cur_pos[0],
                cur_pos[1],
                cur_pos[2],
                cur_vel[0],
                cur_vel[1],
                cur_vel[2],
                phi,
                theta,
                psi,
                cur_ang_vel[0],
                cur_ang_vel[1],
                cur_ang_vel[2],
            ]
        )

        # ------------------------------------------------------------------
        #              Desired 12D reference for hover
        #  For the bonus, the spec wants a regulator around hover:
        #    - target roll, pitch ≈ 0
        #    - small or zero yaw rate
        # ------------------------------------------------------------------
        xd = np.array(
            [
                target_pos[0],
                target_pos[1],
                target_pos[2],
                target_vel[0],
                target_vel[1],
                target_vel[2],
                0.0,
                0.0,
                target_rpy[2],  # keep yaw at desired value
                0.0,
                0.0,
                target_rpy_rates[2],
            ]
        )

        # ------------------------------------------------------------------
        #                        LQR CONTROL LAW
        # ------------------------------------------------------------------
        e = state - xd
        u = -self.K @ e  # u = [delta_u1, tau_phi, tau_theta, tau_psi]

        delta_u1 = u[0]
        tau = u[1:]

        # Total thrust (Newton), keep it positive
        total_thrust = self.hover_thrust + delta_u1
        total_thrust = max(0.0, total_thrust)

        # --------------------------------------------------------------
        #      Map total thrust + torques → motor PWM → RPM
        #  We mimic DSLPIDControl:
        #   1) Convert total_thrust → baseline PWM (all motors equal)
        #   2) Add torque-based PWM offsets via MIXER_MATRIX
        # --------------------------------------------------------------
        # Guard against KF = 0 in URDF
        if self.KF <= 0.0:
            # Fallback: keep old hover PWM hard-coded
            thrust_pwm = 40000.0
        else:
            # Total thrust = 4 * KF * rpm^2  ⇒  rpm = sqrt(total_thrust / (4*KF))
            rpm_hover = math.sqrt(total_thrust / (4.0 * self.KF))
            thrust_pwm = (rpm_hover - self.PWM2RPM_CONST) / self.PWM2RPM_SCALE

        # Clip torques to reasonable limits (similar order as PID)
        tau = np.clip(tau, -3.0e3, 3.0e3)

        pwm = thrust_pwm + np.dot(self.MIXER_MATRIX, tau)
        pwm = np.clip(pwm, self.MIN_PWM, self.MAX_PWM)

        rpm = self.PWM2RPM_SCALE * pwm + self.PWM2RPM_CONST

        # Position error (for logging)
        pos_e = target_pos - cur_pos

        # For logging we return target_rpy as "desired"
        return rpm, pos_e, target_rpy
