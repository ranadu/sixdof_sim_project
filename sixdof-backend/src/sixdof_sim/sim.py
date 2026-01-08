from __future__ import annotations
from dataclasses import dataclass
from typing import Callable, Optional, Dict
import numpy as np

from .state import RigidBodyState
from .integrators import rk4_step
from .math_utils import dcm_bn_from_quat, quat_kinematics, quat_normalize, skew
from .telemetry import TelemetryLog
from .aircraft.simple_aircraft import SimpleAircraft, Controls

@dataclass
class SimConfig:
    dt: float = 0.01
    t_final: float = 10.0

ControlLaw = Callable[[float, RigidBodyState], Controls]

class Simulator:
    def __init__(
        self,
        aircraft: SimpleAircraft,
        cfg: SimConfig,
        control_law: ControlLaw,
        logger: Optional[TelemetryLog] = None,
    ):
        self.ac = aircraft
        self.cfg = cfg
        self.u_of_t = control_law
        self.log = logger if logger is not None else TelemetryLog()

    def dynamics(self, t: float, x: np.ndarray) -> np.ndarray:
        s = RigidBodyState.unpack(x)
        s.normalize()

        u = self.u_of_t(t, s)
        F_b, M_b = self.ac.forces_moments(x, u)

        m = self.ac.p.mass
        I = self.ac.I
        I_inv = self.ac.I_inv

        v_b = s.vel_b
        omega = s.omega_b

        # Translational: vdot = (1/m)F - omega×v
        vdot = (1.0/m) * F_b - np.cross(omega, v_b)

        # Rotational: omegadot = I^-1 (M - omega×(I omega))
        omegadot = I_inv @ (M_b - np.cross(omega, (I @ omega)))

        # Position kinematics: rdot_n = R_bn * v_b
        R_bn = dcm_bn_from_quat(s.q_bn)
        rdot_n = R_bn @ v_b

        # Quaternion kinematics
        qdot = quat_kinematics(s.q_bn, omega)

        xdot = np.hstack([rdot_n, vdot, qdot, omegadot]).astype(float)
        return xdot

    def run(self, x0: RigidBodyState) -> Dict[str, np.ndarray]:
        t = 0.0
        dt = float(self.cfg.dt)
        tf = float(self.cfg.t_final)

        x = x0.pack()
        x[6:10] = quat_normalize(x[6:10])

        # initial log
        F0, M0 = self.ac.forces_moments(x, self.u_of_t(0.0, x0))
        self.log.append(t, x, F0, M0)

        n_steps = int(np.ceil(tf / dt))
        for _ in range(n_steps):
            x = rk4_step(self.dynamics, t, x, dt)
            t += dt
            x[6:10] = quat_normalize(x[6:10])

            s = RigidBodyState.unpack(x)
            F, M = self.ac.forces_moments(x, self.u_of_t(t, s))
            self.log.append(t, x, F, M)

        return self.log.to_arrays()