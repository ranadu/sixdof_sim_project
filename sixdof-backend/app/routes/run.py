from __future__ import annotations

from typing import Any

import numpy as np
from fastapi import APIRouter
from pydantic import BaseModel, Field

router = APIRouter()


class RunRequest(BaseModel):
    scenario: str = Field(default="aileron_pulse")
    dt: float = Field(default=0.01, gt=0.0)
    t_final: float = Field(default=12.0, gt=0.0)
    stride: int = Field(default=2, ge=1)


@router.post("/run")
def run(req: RunRequest) -> dict[str, Any]:
    """
    Demo simulation generator.

    NOTE: This is intentionally deterministic and lightweight for Render Free tier.
    You can swap this out later for your real 6DOF sim core.
    """
    n = int(req.t_final / req.dt) + 1
    t = np.linspace(0.0, req.t_final, n)
    t = t[:: req.stride]
    n2 = t.size

    # Fake-but-plausible-ish signals for demo plotting
    pN = 70.0 * t
    pE = 1.0 * (t**2)
    pD = 1000.0 + 2.0 * (t**2)

    u = 65.0 + 0.2 * (t**2)
    v = 0.2 * np.exp(-t) * np.sin(5 * t)
    w = 0.15 * np.exp(-t) * np.cos(3 * t)

    p = np.zeros(n2)
    q = 0.02 + 0.001 * (t**2)
    r = -0.03 * (1 - np.exp(-t / 2))

    # Aileron pulse
    pulse = (t > 2.0) & (t < 3.0)
    p[pulse] = 0.33
    p[t >= 3.0] = -0.01

    # Quaternion-ish evolution (not physically strict here; you will replace later)
    q0 = 1.0 - 0.0007 * t
    q1 = 0.05 * pulse.astype(float) + 0.12 * (1 - np.exp(-t / 4))
    q2 = 0.04 * (t / req.t_final) ** 1.2
    q3 = -0.01 * t

    Fx = 500.0 * t
    Fy = -4000.0 * (t > 3.0)
    Fz = -1500.0 - 120.0 * (t**2)

    Mx = np.zeros(n2)
    Mx[(t > 2.0) & (t < 2.1)] = 6000.0
    Mx[(t > 3.0) & (t < 3.1)] = -4500.0
    My = 1200.0 * np.exp(-t) * np.cos(4 * t)
    Mz = np.zeros(n2)

    X = np.vstack([pN, pE, pD, u, v, w, p, q, r, q0, q1, q2, q3]).T
    F = np.vstack([Fx, Fy, Fz]).T
    M = np.vstack([Mx, My, Mz]).T

    return {
        "t": t.tolist(),
        "X": X.tolist(),
        "F": F.tolist(),
        "M": M.tolist(),
        "labels": {
            "X": ["pN", "pE", "pD", "u", "v", "w", "p", "q", "r", "q0", "q1", "q2", "q3"],
            "F": ["Fx", "Fy", "Fz"],
            "M": ["Mx", "My", "Mz"],
        },
    }



# from __future__ import annotations

# from fastapi import APIRouter, HTTPException
# from pydantic import BaseModel, Field
# from typing import Any, Dict, Literal, Optional, List
# import numpy as np
# import sys
# from pathlib import Path

# # Allow imports from src/
# ROOT = Path(__file__).resolve().parents[2]
# SRC = ROOT / "src"
# sys.path.insert(0, str(SRC))

# from sixdof_sim.sim import Simulator, SimConfig
# from sixdof_sim.state import RigidBodyState
# from sixdof_sim.math_utils import quat_from_euler
# from sixdof_sim.aircraft.simple_aircraft import SimpleAircraft, SimpleAircraftParams, Controls

# router = APIRouter(prefix="/api/v1", tags=["api"])


# class InitState(BaseModel):
#     pos_ned: List[float] = Field(..., min_length=3, max_length=3)
#     vel_b: List[float] = Field(..., min_length=3, max_length=3)
#     q_bn: Optional[List[float]] = Field(None, min_length=4, max_length=4)
#     euler_rad: Optional[List[float]] = Field(None, min_length=3, max_length=3)
#     omega_b: List[float] = Field(..., min_length=3, max_length=3)


# class ControlSchedule(BaseModel):
#     type: Literal["constant", "aileron_pulse", "elevator_step"] = "constant"
#     de: float = 0.0
#     da: float = 0.0
#     dr: float = 0.0
#     throttle: float = 0.55
#     t_on: float = 1.0
#     t_off: float = 2.0
#     magnitude: float = 0.0


# class SixDofPayload(BaseModel):
#     dt: float = 0.01
#     t_final: float = 10.0
#     downsample_hz: float = 50.0
#     state0: InitState
#     controls: ControlSchedule = ControlSchedule()
#     aircraft: Optional[Dict[str, Any]] = None


# class RunRequest(BaseModel):
#     task: str
#     payload: Dict[str, Any] = Field(default_factory=dict)
#     mode: Literal["sync"] = "sync"


# class RunResponse(BaseModel):
#     ok: bool
#     task: str
#     result: Dict[str, Any]


# def to_state(s0: InitState) -> RigidBodyState:
#     pos = np.array(s0.pos_ned, dtype=float)
#     vel = np.array(s0.vel_b, dtype=float)
#     omg = np.array(s0.omega_b, dtype=float)

#     if s0.q_bn is not None:
#         q = np.array(s0.q_bn, dtype=float)
#     elif s0.euler_rad is not None:
#         phi, th, psi = map(float, s0.euler_rad)
#         q = quat_from_euler(phi, th, psi)
#     else:
#         q = quat_from_euler(0.0, 0.0, 0.0)

#     st = RigidBodyState(pos_ned=pos, vel_b=vel, q_bn=q, omega_b=omg)
#     st.normalize()
#     return st


# def make_control_law(cs: ControlSchedule):
#     def u_of_t(t: float, x: RigidBodyState) -> Controls:
#         if cs.type == "constant":
#             return Controls(de=cs.de, da=cs.da, dr=cs.dr, throttle=cs.throttle)
#         if cs.type == "aileron_pulse":
#             da = cs.magnitude if (cs.t_on <= t <= cs.t_off) else 0.0
#             return Controls(de=cs.de, da=da, dr=cs.dr, throttle=cs.throttle)
#         if cs.type == "elevator_step":
#             de = cs.magnitude if (t >= cs.t_on) else cs.de
#             return Controls(de=de, da=cs.da, dr=cs.dr, throttle=cs.throttle)
#         return Controls()
#     return u_of_t


# @router.post("/run", response_model=RunResponse)
# def run(req: RunRequest) -> RunResponse:
#     if req.task != "sixdof_simulate":
#         raise HTTPException(status_code=400, detail=f"Unknown task '{req.task}'")

#     payload = SixDofPayload(**req.payload)

#     p = SimpleAircraftParams()
#     if payload.aircraft:
#         p = SimpleAircraftParams(**{**p.__dict__, **payload.aircraft})

#     ac = SimpleAircraft(p)
#     sim = Simulator(ac, SimConfig(dt=payload.dt, t_final=payload.t_final), make_control_law(payload.controls))
#     out = sim.run(to_state(payload.state0))

#     stride = max(1, int((1.0 / max(payload.downsample_hz, 1.0)) / payload.dt))

#     t = out["t"][::stride]
#     X = out["X"][::stride]
#     F = out["F"][::stride]
#     M = out["M"][::stride]

#     return RunResponse(
#         ok=True,
#         task=req.task,
#         result={
#             "t": t.astype(float).tolist(),
#             "X": X.astype(float).tolist(),
#             "F": F.astype(float).tolist(),
#             "M": M.astype(float).tolist(),
#             "meta": {
#                 "dt": payload.dt,
#                 "t_final": payload.t_final,
#                 "downsample_hz": payload.downsample_hz,
#                 "stride": stride,
#                 "state_format": "pos(3), vel_b(3), q_bn(4), omega(3) => 13 cols",
#             },
#         },
#     )