from __future__ import annotations
import numpy as np
from .constants import EPS

def skew(w: np.ndarray) -> np.ndarray:
    wx, wy, wz = float(w[0]), float(w[1]), float(w[2])
    return np.array([[0, -wz, wy],
                     [wz, 0, -wx],
                     [-wy, wx, 0]], dtype=float)

def quat_normalize(q: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(q))
    if n < EPS:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    return q / n

def quat_mul(q: np.ndarray, p: np.ndarray) -> np.ndarray:
    # scalar-first Hamilton product
    q0,q1,q2,q3 = q
    p0,p1,p2,p3 = p
    return np.array([
        q0*p0 - q1*p1 - q2*p2 - q3*p3,
        q0*p1 + q1*p0 + q2*p3 - q3*p2,
        q0*p2 - q1*p3 + q2*p0 + q3*p1,
        q0*p3 + q1*p2 - q2*p1 + q3*p0,
    ], dtype=float)

def quat_from_euler(phi: float, theta: float, psi: float) -> np.ndarray:
    # 3-2-1 yaw-pitch-roll
    c1, s1 = np.cos(psi/2), np.sin(psi/2)
    c2, s2 = np.cos(theta/2), np.sin(theta/2)
    c3, s3 = np.cos(phi/2), np.sin(phi/2)
    q0 = c1*c2*c3 + s1*s2*s3
    q1 = c1*c2*s3 - s1*s2*c3
    q2 = c1*s2*c3 + s1*c2*s3
    q3 = s1*c2*c3 - c1*s2*s3
    return quat_normalize(np.array([q0,q1,q2,q3], dtype=float))

def euler_from_quat(q: np.ndarray) -> np.ndarray:
    q0,q1,q2,q3 = q
    # roll
    sinr = 2*(q0*q1 + q2*q3)
    cosr = 1 - 2*(q1*q1 + q2*q2)
    phi = np.arctan2(sinr, cosr)
    # pitch
    sinp = 2*(q0*q2 - q3*q1)
    if abs(sinp) >= 1:
        theta = np.sign(sinp) * (np.pi/2)
    else:
        theta = np.arcsin(sinp)
    # yaw
    siny = 2*(q0*q3 + q1*q2)
    cosy = 1 - 2*(q2*q2 + q3*q3)
    psi = np.arctan2(siny, cosy)
    return np.array([phi, theta, psi], dtype=float)

def dcm_bn_from_quat(q: np.ndarray) -> np.ndarray:
    # body->NED (b to n) from quaternion (scalar-first)
    q0,q1,q2,q3 = q
    R = np.array([
        [1-2*(q2*q2+q3*q3),   2*(q1*q2+q0*q3),   2*(q1*q3-q0*q2)],
        [2*(q1*q2-q0*q3),     1-2*(q1*q1+q3*q3), 2*(q2*q3+q0*q1)],
        [2*(q1*q3+q0*q2),     2*(q2*q3-q0*q1),   1-2*(q1*q1+q2*q2)]
    ], dtype=float)
    return R

def quat_kinematics(q: np.ndarray, omega_b: np.ndarray) -> np.ndarray:
    # qdot = 0.5 * Omega(omega) * q
    p,qb,r = omega_b
    Omega = np.array([
        [0, -p, -qb, -r],
        [p,  0,  r, -qb],
        [qb, -r, 0,  p],
        [r,  qb, -p, 0],
    ], dtype=float)
    return 0.5 * Omega @ q