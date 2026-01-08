from fastapi import APIRouter
from app.core.config import settings

router = APIRouter()

@router.get("/meta")
def meta():
    return {"name": settings.app_name, "version": settings.version, "env": settings.env}