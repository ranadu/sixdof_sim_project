from __future__ import annotations

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field
from typing import Any, Dict, Literal, Optional, List
import numpy as np
import sys
from pathlib import Path

# Ensure we can import your sim core from ../src
PROJECT_ROOT = Path(__file__).resolve().parents[2]  # .../app/routes -> .../
SRC_PATH = PROJECT_ROOT / "src"
if SRC_PATH.exists():
    sys.path.insert(0, str(SRC_PATH))

from sixdof_sim.sim import Simulator, SimConfig
from sixdof_sim.state import RigidBodyState
from sixdof_sim.math_utils import quat_from_euler
from sixdof_sim.aircraft.simple_aircraft import SimpleAircraft, SimpleAircraftParams, Controls


router = APIRouter(prefix="/api/v1", tags=["api"])


class SixDofInitState(BaseModel):
    pos_ned: List[float] = Field(..., min_length=3, max_length=3)
    vel_b: List[float] = Field(..., min_length=3, max_length=3)
    q_bn: Optional[List[float]] = Field(None, min_length=4, max_length=4)  # scalar-first
    euler_rad: Optional[List[float]] = Field(None, min_length=3, max_length=3)  # [phi,theta,psi]
    omega_b: List[float] = Field(..., min_length=3, max_length=3)


class SixDofControlSchedule(BaseModel):
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
    downsample_hz: float = 50.0  # keeps payload small for web
    state0: SixDofInitState
    controls: SixDofControlSchedule = SixDofControlSchedule()
    aircraft: Optional[Dict[str, Any]] = None  # override SimpleAircraftParams if needed


class RunRequest(BaseModel):
    task: str = Field(..., description="Task name e.g. 'sixdof_simulate'")
    payload: Dict[str, Any] = Field(default_factory=dict)
    mode: Literal["sync"] = "sync"


class RunResponse(BaseModel):
    ok: bool
    task: str
    result: Dict[str, Any]


def _to_state(s0: SixDofInitState) -> RigidBodyState:
    pos = np.array(s0.pos_ned, dtype=float)
    vel = np.array(s0.vel_b, dtype=float)
    omega = np.array(s0.omega_b, dtype=float)

    if s0.q_bn is not None:
        q = np.array(s0.q_bn, dtype=float)
    elif s0.euler_rad is not None:
        phi, theta, psi = float(s0.euler_rad[0]), float(s0.euler_rad[1]), float(s0.euler_rad[2])
        q = quat_from_euler(phi, theta, psi)
    else:
        q = quat_from_euler(0.0, 0.0, 0.0)

    x0 = RigidBodyState(pos_ned=pos, vel_b=vel, q_bn=q, omega_b=omega)
    x0.normalize()
    return x0


def _make_control_law(cs: SixDofControlSchedule):
    def u_of_t(t: float, x: RigidBodyState) -> Controls:
        if cs.type == "constant":
            return Controls(de=cs.de, da=cs.da, dr=cs.dr, throttle=cs.throttle)
        if cs.type == "aileron_pulse":
            da = cs.magnitude if (cs.t_on <= t <= cs.t_off) else 0.0
            return Controls(de=cs.de, da=da, dr=cs.dr, throttle=cs.throttle)
        if cs.type == "elevator_step":
            de = cs.magnitude if (t >= cs.t_on) else cs.de
            return Controls(de=de, da=cs.da, dr=cs.dr, throttle=cs.throttle)
        return Controls(de=0.0, da=0.0, dr=0.0, throttle=0.55)
    return u_of_t


@router.post("/run", response_model=RunResponse)
def run(req: RunRequest) -> RunResponse:
    if req.task != "sixdof_simulate":
        raise HTTPException(status_code=400, detail=f"Unknown task '{req.task}'")

    payload = SixDofPayload(**req.payload)

    # Aircraft params
    p = SimpleAircraftParams()
    if payload.aircraft:
        p = SimpleAircraftParams(**{**p.__dict__, **payload.aircraft})

    ac = SimpleAircraft(p)
    cfg = SimConfig(dt=payload.dt, t_final=payload.t_final)
    sim = Simulator(ac, cfg, _make_control_law(payload.controls), logger=None)

    out = sim.run(_to_state(payload.state0))

    # Downsample to keep web payload manageable
    stride = max(1, int((1.0 / max(payload.downsample_hz, 1.0)) / payload.dt))
    t = out["t"][::stride]
    X = out["X"][::stride]
    F = out["F"][::stride]
    M = out["M"][::stride]

    result = {
        "t": t.astype(float).tolist(),
        "X": X.astype(float).tolist(),
        "F": F.astype(float).tolist(),
        "M": M.astype(float).tolist(),
        "meta": {
            "dt": payload.dt,
            "t_final": payload.t_final,
            "downsample_hz": payload.downsample_hz,
            "stride": stride,
            "state_format": "pos_ned(3), vel_b(3), q_bn(4), omega_b(3) => 13 columns",
        },
    }

    return RunResponse(ok=True, task=req.task, result=result)



# from __future__ import annotations

# from fastapi import APIRouter, HTTPException
# from pydantic import BaseModel, Field
# from typing import Any, Dict, Literal

# router = APIRouter(prefix="/api/v1", tags=["api"])


# class RunRequest(BaseModel):
#     task: str = Field(..., description="Task name (e.g., 'sixdof_simulate')")
#     payload: Dict[str, Any] = Field(default_factory=dict)
#     mode: Literal["sync"] = "sync"


# class RunResponse(BaseModel):
#     ok: bool
#     task: str
#     result: Dict[str, Any]


# @router.post("/run", response_model=RunResponse)
# def run(req: RunRequest) -> RunResponse:
#     # Template mode: backend is deployable even before you plug in the sim core
#     if req.task == "sixdof_simulate":
#         raise HTTPException(
#             status_code=501,
#             detail="sixdof_simulate not installed yet. Add src/sixdof_sim then redeploy.",
#         )

#     return RunResponse(
#         ok=True,
#         task=req.task,
#         result={
#             "echo": req.payload,
#             "message": f"Template backend is live. Task '{req.task}' executed (echo only).",
#         },
#     )