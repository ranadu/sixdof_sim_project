from __future__ import annotations

from pydantic_settings import BaseSettings, SettingsConfigDict


class Settings(BaseSettings):
    """
    App configuration.

    Environment variables:
      - APP_NAME
      - VERSION
      - ALLOWED_ORIGINS  (comma-separated)
      - CORS_ALLOW_ALL   ("true"/"false")
    """
    model_config = SettingsConfigDict(env_file=".env", extra="ignore")

    app_name: str = "sixdof_sim_project"
    version: str = "0.1.0"

    # Comma separated: "https://a.vercel.app,https://b.vercel.app"
    allowed_origins: str = ""

    # When true, allow "*" (best for demos to stop CORS pain)
    cors_allow_all: bool = True


settings = Settings()