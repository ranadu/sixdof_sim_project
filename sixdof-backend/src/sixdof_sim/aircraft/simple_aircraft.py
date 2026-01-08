from __future__ import annotations
from dataclasses import dataclass
import numpy as np
from ..constants import G0
from ..math_utils import dcm_bn_from_quat

@dataclass
class Controls:
    de: float = 0.0       # elevator [rad]
    da: float = 0.0       # aileron [rad]
    dr: float = 0.0       # rudder  [rad]
    throttle: float = 0.55

@dataclass
class SimpleAircraftParams:
    # mass/inertia
    mass: float = 1200.0
    Ixx: float = 950.0
    Iyy: float = 1400.0
    Izz: float = 1800.0

    # geometry / aero reference
    S: float = 16.2
    b: float = 10.9
    c: float = 1.5

    # atmosphere (constant)
    rho: float = 1.225

    # thrust
    max_thrust: float = 3000.0  # N

    # linear-ish stability derivatives (toy but plausible)
    CL0: float = 0.25
    CL_alpha: float = 5.0
    CL_q: float = 3.5
    CL_de: float = 0.6

    CD0: float = 0.03
    CD_k: float = 0.06

    CY_beta: float = -0.8
    CY_dr: float = 0.2

    Cl_beta: float = -0.12
    Cl_p: float = -0.5
    Cl_r: float = 0.25
    Cl_da: float = 0.08
    Cl_dr: float = 0.02

    Cm0: float = 0.02
    Cm_alpha: float = -1.2
    Cm_q: float = -8.0
    Cm_de: float = -1.0

    Cn_beta: float = 0.18
    Cn_p: float = -0.06
    Cn_r: float = -0.2
    Cn_da: float = 0.01
    Cn_dr: float = -0.08

class SimpleAircraft:
    def __init__(self, p: SimpleAircraftParams):
        self.p = p
        self.I = np.diag([p.Ixx, p.Iyy, p.Izz])
        self.I_inv = np.diag([1.0/p.Ixx, 1.0/p.Iyy, 1.0/p.Izz])

    def forces_moments(self, x: np.ndarray, u: Controls) -> tuple[np.ndarray, np.ndarray]:
        # state vector layout: pos(3), vel_b(3), q_bn(4), omega(3)
        vel_b = x[3:6]
        q_bn = x[6:10]
        omega = x[10:13]
        p_rate, q_rate, r_rate = omega

        V = float(np.linalg.norm(vel_b))
        V = max(V, 0.1)

        # angles
        alpha = np.arctan2(vel_b[2], vel_b[0])  # w/u
        beta = np.arcsin(np.clip(vel_b[1]/V, -1.0, 1.0))

        qbar = 0.5 * self.p.rho * V*V

        # nondimensional rates
        p_hat = p_rate * self.p.b / (2*V)
        q_hat = q_rate * self.p.c / (2*V)
        r_hat = r_rate * self.p.b / (2*V)

        # coefficients
        CL = self.p.CL0 + self.p.CL_alpha*alpha + self.p.CL_q*q_hat + self.p.CL_de*u.de
        CD = self.p.CD0 + self.p.CD_k*(CL*CL)
        CY = self.p.CY_beta*beta + self.p.CY_dr*u.dr

        Cl = (self.p.Cl_beta*beta + self.p.Cl_p*p_hat + self.p.Cl_r*r_hat
              + self.p.Cl_da*u.da + self.p.Cl_dr*u.dr)
        Cm = self.p.Cm0 + self.p.Cm_alpha*alpha + self.p.Cm_q*q_hat + self.p.Cm_de*u.de
        Cn = (self.p.Cn_beta*beta + self.p.Cn_p*p_hat + self.p.Cn_r*r_hat
              + self.p.Cn_da*u.da + self.p.Cn_dr*u.dr)

        # wind axes forces
        L = qbar * self.p.S * CL
        D = qbar * self.p.S * CD
        Y = qbar * self.p.S * CY

        # convert to body axes (approx)
        ca, sa = np.cos(alpha), np.sin(alpha)
        # Xb/Zb from D and L
        X_aero = -D*ca + -L*sa
        Z_aero =  D*sa + -L*ca
        Y_aero = Y

        F_aero_b = np.array([X_aero, Y_aero, Z_aero], dtype=float)

        # thrust in +X body
        T = np.clip(u.throttle, 0.0, 1.0) * self.p.max_thrust
        F_thrust_b = np.array([T, 0.0, 0.0], dtype=float)

        # gravity in body frame: g_n = [0,0,g] in NED (down positive)
        R_bn = dcm_bn_from_quat(q_bn)     # body->ned
        R_nb = R_bn.T                    # ned->body
        g_n = np.array([0.0, 0.0, G0], dtype=float)
        F_grav_b = self.p.mass * (R_nb @ g_n)

        F_total_b = F_aero_b + F_thrust_b + F_grav_b

        # moments
        Mx = qbar * self.p.S * self.p.b * Cl
        My = qbar * self.p.S * self.p.c * Cm
        Mz = qbar * self.p.S * self.p.b * Cn
        M_total_b = np.array([Mx, My, Mz], dtype=float)

        return F_total_b, M_total_b