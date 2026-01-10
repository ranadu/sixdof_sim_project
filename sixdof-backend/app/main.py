from __future__ import annotations

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from app.core.config import settings
from app.routes.health import router as health_router
from app.routes.meta import router as meta_router
from app.routes.run import router as run_router


def build_allowed_origins() -> list[str]:
    if settings.cors_allow_all:
        return ["*"]
    if not settings.allowed_origins.strip():
        return []
    return [o.strip() for o in settings.allowed_origins.split(",") if o.strip()]


app = FastAPI(title=settings.app_name, version=settings.version)

# ---- CORS ----
origins = build_allowed_origins()
app.add_middleware(
    CORSMiddleware,
    allow_origins=origins if origins else [],  # empty means "block all"
    allow_credentials=False,
    allow_methods=["*"],
    allow_headers=["*"],
)

# ---- Routes ----
@app.get("/")
def home():
    return {
        "status": "ok",
        "message": "Backend is live",
        "try": ["/health", "/meta", "/docs", "/routes", "/api/v1/run"],
        "cors": {
            "cors_allow_all": settings.cors_allow_all,
            "allowed_origins": origins,
        },
    }


@app.get("/routes")
def routes():
    return [
        {"path": r.path, "name": r.name, "methods": sorted(list(r.methods or []))}
        for r in app.router.routes
    ]


# Important: mount run_router under /api/v1
app.include_router(health_router)
app.include_router(meta_router)
app.include_router(run_router, prefix="/api/v1", tags=["sim"])