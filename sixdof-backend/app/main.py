from __future__ import annotations

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from app.core.config import settings
from app.core.logging import setup_logging
from app.routes.health import router as health_router
from app.routes.meta import router as meta_router
from app.routes.run import router as run_router

setup_logging()

app = FastAPI(title=settings.app_name, version=settings.version)

# CORS (lock this to your Vercel domain later)
origins = [o.strip() for o in settings.allowed_origins.split(",")] if settings.allowed_origins else ["*"]
app.add_middleware(
    CORSMiddleware,
    allow_origins=origins if origins != ["*"] else ["*"],
    allow_credentials=False,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(health_router)
app.include_router(meta_router)
app.include_router(run_router)