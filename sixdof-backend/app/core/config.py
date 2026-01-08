from __future__ import annotations
from pydantic import BaseModel
import os


class Settings(BaseModel):
    env: str = os.getenv("ENV", "development")
    allowed_origins: str = os.getenv("ALLOWED_ORIGINS", "*")
    app_name: str = os.getenv("APP_NAME", "6DOF Backend")
    version: str = os.getenv("APP_VERSION", "0.1.0")


settings = Settings()