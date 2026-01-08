from __future__ import annotations

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field
from typing import Any, Dict, Literal, Optional, List
import numpy as np
import sys
from pathlib import Path

# Allow imports from src/
ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "src"
sys.path.insert(0, str(SRC))

from sixdof_sim.sim import Simulator, SimConfig
from sixdof_sim.state import RigidBodyState
from sixdof_sim.math_utils import quat_from_euler
from sixdof_sim.aircraft.simple_aircraft import SimpleAircraft, SimpleAircraftParams, Controls

router = APIRouter(prefix="/api/v1", tags=["api"])


class InitState(BaseModel):
    pos_ned: List[float] = Field(..., min_length=3, max_length=3)
    vel_b: List[float] = Field(..., min_length=3, max_length=3)
    q_bn: Optional[List[float]] = Field(None, min_length=4, max_length=4)
    euler_rad: Optional[List[float]] = Field(None, min_length=3, max_length=3)
    omega_b: List[float] = Field(..., min_length=3, max_length=3)


class ControlSchedule(BaseModel):
    type: Literal["constant", "aileron_pulse", "elevator_step"] = "constant"
    de: float = 0.0
    da: float = 0.0
    dr: float = 0.0
    throttle: float = 0.55
    t_on: float = 1.0
    t_off: float = 2.0
    magnitude: float = 0.0


class SixDofPayload(BaseModel):
    dt: float = 0.01
    t_final: float = 10.0
    downsample_hz: float = 50.0
    state0: InitState
    controls: ControlSchedule = ControlSchedule()
    aircraft: Optional[Dict[str, Any]] = None


class RunRequest(BaseModel):
    task: str
    payload: Dict[str, Any] = Field(default_factory=dict)
    mode: Literal["sync"] = "sync"


class RunResponse(BaseModel):
    ok: bool
    task: str
    result: Dict[str, Any]


def to_state(s0: InitState) -> RigidBodyState:
    pos = np.array(s0.pos_ned, dtype=float)
    vel = np.array(s0.vel_b, dtype=float)
    omg = np.array(s0.omega_b, dtype=float)

    if s0.q_bn is not None:
        q = np.array(s0.q_bn, dtype=float)
    elif s0.euler_rad is not None:
        phi, th, psi = map(float, s0.euler_rad)
        q = quat_from_euler(phi, th, psi)
    else:
        q = quat_from_euler(0.0, 0.0, 0.0)

    st = RigidBodyState(pos_ned=pos, vel_b=vel, q_bn=q, omega_b=omg)
    st.normalize()
    return st


def make_control_law(cs: ControlSchedule):
    def u_of_t(t: float, x: RigidBodyState) -> Controls:
        if cs.type == "constant":
            return Controls(de=cs.de, da=cs.da, dr=cs.dr, throttle=cs.throttle)
        if cs.type == "aileron_pulse":
            da = cs.magnitude if (cs.t_on <= t <= cs.t_off) else 0.0
            return Controls(de=cs.de, da=da, dr=cs.dr, throttle=cs.throttle)
        if cs.type == "elevator_step":
            de = cs.magnitude if (t >= cs.t_on) else cs.de
            return Controls(de=de, da=cs.da, dr=cs.dr, throttle=cs.throttle)
        return Controls()
    return u_of_t


@router.post("/run", response_model=RunResponse)
def run(req: RunRequest) -> RunResponse:
    if req.task != "sixdof_simulate":
        raise HTTPException(status_code=400, detail=f"Unknown task '{req.task}'")

    payload = SixDofPayload(**req.payload)

    p = SimpleAircraftParams()
    if payload.aircraft:
        p = SimpleAircraftParams(**{**p.__dict__, **payload.aircraft})

    ac = SimpleAircraft(p)
    sim = Simulator(ac, SimConfig(dt=payload.dt, t_final=payload.t_final), make_control_law(payload.controls))
    out = sim.run(to_state(payload.state0))

    stride = max(1, int((1.0 / max(payload.downsample_hz, 1.0)) / payload.dt))

    t = out["t"][::stride]
    X = out["X"][::stride]
    F = out["F"][::stride]
    M = out["M"][::stride]

    return RunResponse(
        ok=True,
        task=req.task,
        result={
            "t": t.astype(float).tolist(),
            "X": X.astype(float).tolist(),
            "F": F.astype(float).tolist(),
            "M": M.astype(float).tolist(),
            "meta": {
                "dt": payload.dt,
                "t_final": payload.t_final,
                "downsample_hz": payload.downsample_hz,
                "stride": stride,
                "state_format": "pos(3), vel_b(3), q_bn(4), omega(3) => 13 cols",
            },
        },
    )