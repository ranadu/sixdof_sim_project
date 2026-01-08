from __future__ import annotations
from dataclasses import dataclass
import numpy as np
from .math_utils import quat_normalize

@dataclass
class RigidBodyState:
    # Position in NED [m]
    pos_ned: np.ndarray  # (3,)
    # Velocity in body axes [m/s]
    vel_b: np.ndarray    # (3,)
    # Quaternion body->NED scalar-first
    q_bn: np.ndarray     # (4,)
    # Body rates [rad/s]
    omega_b: np.ndarray  # (3,)

    def copy(self) -> "RigidBodyState":
        return RigidBodyState(
            pos_ned=self.pos_ned.copy(),
            vel_b=self.vel_b.copy(),
            q_bn=self.q_bn.copy(),
            omega_b=self.omega_b.copy(),
        )

    def normalize(self) -> None:
        self.q_bn = quat_normalize(self.q_bn)

    def pack(self) -> np.ndarray:
        return np.hstack([self.pos_ned, self.vel_b, self.q_bn, self.omega_b]).astype(float)

    @staticmethod
    def unpack(x: np.ndarray) -> "RigidBodyState":
        x = np.asarray(x, dtype=float).reshape(-1)
        return RigidBodyState(
            pos_ned=x[0:3].copy(),
            vel_b=x[3:6].copy(),
            q_bn=x[6:10].copy(),
            omega_b=x[10:13].copy(),
        )