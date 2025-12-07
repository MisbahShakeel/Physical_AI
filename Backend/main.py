from contextlib import asynccontextmanager
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import os
from pydantic_settings import BaseSettings


# Define configuration
class Settings(BaseSettings):
    openai_api_key: str = os.getenv("OPENAI_API_KEY", "")
    qdrant_url: str = os.getenv("QDRANT_URL", "http://localhost:6333")
    qdrant_api_key: str = os.getenv("QDRANT_API_KEY", "")
    postgres_dsn: str = os.getenv("POSTGRES_DSN", "postgresql://user:password@localhost/dbname")

    class Config:
        env_file = ".env"


settings = Settings()


@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup
    app.state.config = settings
    yield
    # Shutdown


# Create FastAPI app
app = FastAPI(
    title="Textbook RAG API",
    description="Retrieval-Augmented Generation API for technical textbook",
    version="1.0.0",
    lifespan=lifespan
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# Import and include routers after app is created to avoid circular imports
def include_routers():
    try:
        # Try to import the main API routes
        from fastapi_endpoints import app as api_app

        # Add the routes from the imported app
        for route in api_app.routes:
            app.router.routes.append(route)
    except ImportError as e:
        print(f"Could not import API routes: {e}")
        # Define minimal routes for testing
        @app.get("/")
        def read_root():
            return {"message": "Textbook RAG API is running!"}

        @app.get("/health")
        def health_check():
            return {"status": "healthy", "timestamp": "2024-01-01T00:00:00Z"}


# Include routers
include_routers()


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)