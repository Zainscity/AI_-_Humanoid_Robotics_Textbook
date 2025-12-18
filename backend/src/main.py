from fastapi import FastAPI, Request
from fastapi.responses import JSONResponse
from fastapi.middleware.cors import CORSMiddleware
from src.api.endpoints import query, history
from src.api import auth
from .core.logging import get_logger

logger = get_logger(__name__)

app = FastAPI()

origins = [
    "http://localhost",
    "http://localhost:3000",
    "https://ai-humanoid-robotics-textbook-a9c0uvvkm-zainscitys-projects.vercel.app",
    "https://ai-humanoid-robotics-textbook-three.vercel.app",
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.exception_handler(Exception)
async def generic_exception_handler(request: Request, exc: Exception):
    logger.error(f"An unexpected error occurred: {exc}", exc_info=True)
    return JSONResponse(
        status_code=500,
        content={"message": "An unexpected error occurred. Please try again later."},
    )

app.include_router(auth.router, prefix="/auth", tags=["auth"])
app.include_router(query.router, prefix="/query", tags=["query"])
app.include_router(history.router, prefix="/history", tags=["history"])

@app.get("/health")
def health_check():
    return {"status": "ok"}
