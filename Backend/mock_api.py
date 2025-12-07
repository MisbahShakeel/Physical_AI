"""
Mock API for Textbook RAG System - For frontend integration testing
This provides a simplified version that doesn't require external services
"""
from fastapi import FastAPI, HTTPException, Depends
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field
from typing import List, Optional, Dict, Any
import uuid
from datetime import datetime
from contextlib import asynccontextmanager
import random
import json


# Pydantic models
class QueryRequest(BaseModel):
    query: str = Field(..., min_length=1, max_length=2000)
    session_id: Optional[str] = None
    selected_text: Optional[str] = Field(None, max_length=5000)
    top_k: int = Field(10, ge=1, le=20)
    temperature: float = Field(0.1, ge=0.0, le=1.0)


class QueryResponse(BaseModel):
    response: str
    citations: List[Dict[str, Any]]
    search_mode: str
    latency_ms: int
    query_id: str


class HealthResponse(BaseModel):
    status: str
    timestamp: str
    services: Dict[str, str]


def generate_mock_citations(query: str) -> List[Dict[str, Any]]:
    """Generate mock citations based on the query"""
    modules = ["Module 1: ROS 2 Fundamentals", "Module 2: Simulation Environments",
               "Module 3: NVIDIA Isaac Integration", "Module 4: Vision-Language-Action Systems"]
    chapters = ["Introduction", "Core Concepts", "Implementation", "Advanced Topics"]

    citations = []
    for i in range(min(3, len(modules))):
        citations.append({
            "chapter_title": random.choice(chapters),
            "section_title": f"Section {random.randint(1, 5)}",
            "page_number": random.randint(10, 200),
            "module": random.choice(modules)
        })

    return citations


# Mock data for responses
MOCK_RESPONSES = [
    "Based on the textbook content, ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It provides a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms.",
    "Gazebo is a 3D simulation environment that enables accurate and efficient simulation of robots and environments. It provides physics simulation, sensor simulation, and realistic rendering capabilities essential for robotics development.",
    "NVIDIA Isaac is a robotics platform that accelerates AI-powered robotics development. It includes Isaac Sim for photorealistic simulation, Isaac ROS for hardware-accelerated perception, and Isaac Lab for robot learning applications.",
    "Vision-Language-Action (VLA) systems represent an emerging paradigm in robotics where visual input, natural language understanding, and action execution are tightly integrated to enable more intuitive human-robot interaction."
]


@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup
    print("Mock API starting up...")

    # Initialize mock services
    app.state.mock_initialized = True

    yield

    # Shutdown
    print("Mock API shutting down...")


# Create FastAPI app
app = FastAPI(
    title="Textbook RAG Mock API",
    description="Mock Retrieval-Augmented Generation API for technical textbook (for frontend integration)",
    version="1.0.0",
    lifespan=lifespan
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Allow all origins for development
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


def generate_mock_citations(query: str) -> List[Dict[str, Any]]:
    """Generate mock citations based on the query"""
    modules = ["Module 1: ROS 2 Fundamentals", "Module 2: Simulation Environments",
               "Module 3: NVIDIA Isaac Integration", "Module 4: Vision-Language-Action Systems"]
    chapters = ["Introduction", "Core Concepts", "Implementation", "Advanced Topics"]

    citations = []
    for i in range(min(3, len(modules))):
        citations.append({
            "chapter_title": random.choice(chapters),
            "section_title": f"Section {random.randint(1, 5)}",
            "page_number": random.randint(10, 200),
            "module": random.choice(modules)
        })

    return citations


@app.post("/query", response_model=QueryResponse)
async def query_documents(request: QueryRequest):
    """
    Mock endpoint to query the textbook knowledge base
    """
    start_time = datetime.now()

    try:
        # Create or validate session
        session_id = request.session_id or str(uuid.uuid4())

        # Generate mock response based on query
        response_text = random.choice(MOCK_RESPONSES)
        if "ros" in request.query.lower():
            response_text = "Based on the textbook content, ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It provides a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms. Key concepts include nodes, topics, services, and actions for inter-process communication."
        elif "simulation" in request.query.lower() or "gazebo" in request.query.lower():
            response_text = "Gazebo is a 3D simulation environment that enables accurate and efficient simulation of robots and environments. It provides physics simulation, sensor simulation, and realistic rendering capabilities essential for robotics development. Gazebo helps in testing algorithms before deploying to real robots, reducing development time and costs."
        elif "nvidia" in request.query.lower() or "isaac" in request.query.lower():
            response_text = "NVIDIA Isaac is a robotics platform that accelerates AI-powered robotics development. It includes Isaac Sim for photorealistic simulation, Isaac ROS for hardware-accelerated perception, and Isaac Lab for robot learning applications. The platform leverages GPU acceleration for computationally intensive tasks like computer vision and deep learning."
        elif "vision" in request.query.lower() or "language" in request.query.lower() or "action" in request.query.lower():
            response_text = "Vision-Language-Action (VLA) systems represent an emerging paradigm in robotics where visual input, natural language understanding, and action execution are tightly integrated to enable more intuitive human-robot interaction. These systems allow robots to understand complex commands expressed in natural language and act appropriately in dynamic environments."

        # Calculate latency
        latency_ms = int((datetime.now() - start_time).total_seconds() * 1000)

        return QueryResponse(
            response=response_text,
            citations=generate_mock_citations(request.query),
            search_mode="global" if not request.selected_text else "selected_text",
            latency_ms=latency_ms,
            query_id=str(uuid.uuid4())
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Query failed: {str(e)}")


@app.get("/health", response_model=HealthResponse)
async def health_check():
    """
    Health check endpoint to verify system status
    """
    try:
        return HealthResponse(
            status="healthy",
            timestamp=datetime.now().isoformat(),
            services={
                "mock_service": "operational",
                "database": "mocked",
                "external_apis": "mocked"
            }
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Health check failed: {str(e)}")


@app.get("/")
async def root():
    return {"message": "Textbook RAG Mock API is running!"}


# Additional endpoints for compatibility
@app.post("/ingest")
async def ingest_documents():
    """Mock ingest endpoint"""
    return {"status": "success", "documents_processed": 0, "chunks_created": 0}


@app.post("/embed")
async def generate_embedding(text: str):
    """Mock embedding endpoint"""
    return {"embedding": [random.random() for _ in range(1536)], "model": "text-embedding-3-small", "tokens_used": len(text.split())}


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)


@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup
    print("Mock API starting up...")

    # Initialize mock services
    app.state.mock_initialized = True

    yield

    # Shutdown
    print("Mock API shutting down...")


# Create FastAPI app
app = FastAPI(
    title="Textbook RAG Mock API",
    description="Mock Retrieval-Augmented Generation API for technical textbook (for frontend integration)",
    version="1.0.0",
    lifespan=lifespan
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Allow all origins for development
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


def generate_mock_citations(query: str) -> List[Dict[str, Any]]:
    """Generate mock citations based on the query"""
    modules = ["Module 1: ROS 2 Fundamentals", "Module 2: Simulation Environments",
               "Module 3: NVIDIA Isaac Integration", "Module 4: Vision-Language-Action Systems"]
    chapters = ["Introduction", "Core Concepts", "Implementation", "Advanced Topics"]

    citations = []
    for i in range(min(3, len(modules))):
        citations.append({
            "chapter_title": random.choice(chapters),
            "section_title": f"Section {random.randint(1, 5)}",
            "page_number": random.randint(10, 200),
            "module": random.choice(modules)
        })

    return citations


@app.post("/query", response_model=QueryResponse)
async def query_documents(request: QueryRequest):
    """
    Mock endpoint to query the textbook knowledge base
    """
    start_time = datetime.now()

    try:
        # Create or validate session
        session_id = request.session_id or str(uuid.uuid4())

        # Generate mock response based on query
        response_text = random.choice(MOCK_RESPONSES)
        if "ros" in request.query.lower():
            response_text = "Based on the textbook content, ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It provides a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms. Key concepts include nodes, topics, services, and actions for inter-process communication."
        elif "simulation" in request.query.lower() or "gazebo" in request.query.lower():
            response_text = "Gazebo is a 3D simulation environment that enables accurate and efficient simulation of robots and environments. It provides physics simulation, sensor simulation, and realistic rendering capabilities essential for robotics development. Gazebo helps in testing algorithms before deploying to real robots, reducing development time and costs."
        elif "nvidia" in request.query.lower() or "isaac" in request.query.lower():
            response_text = "NVIDIA Isaac is a robotics platform that accelerates AI-powered robotics development. It includes Isaac Sim for photorealistic simulation, Isaac ROS for hardware-accelerated perception, and Isaac Lab for robot learning applications. The platform leverages GPU acceleration for computationally intensive tasks like computer vision and deep learning."
        elif "vision" in request.query.lower() or "language" in request.query.lower() or "action" in request.query.lower():
            response_text = "Vision-Language-Action (VLA) systems represent an emerging paradigm in robotics where visual input, natural language understanding, and action execution are tightly integrated to enable more intuitive human-robot interaction. These systems allow robots to understand complex commands expressed in natural language and act appropriately in dynamic environments."

        # Calculate latency
        latency_ms = int((datetime.now() - start_time).total_seconds() * 1000)

        return QueryResponse(
            response=response_text,
            citations=generate_mock_citations(request.query),
            search_mode="global" if not request.selected_text else "selected_text",
            latency_ms=latency_ms,
            query_id=str(uuid.uuid4())
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Query failed: {str(e)}")


@app.get("/health", response_model=HealthResponse)
async def health_check():
    """
    Health check endpoint to verify system status
    """
    try:
        return HealthResponse(
            status="healthy",
            timestamp=datetime.now().isoformat(),
            services={
                "mock_service": "operational",
                "database": "mocked",
                "external_apis": "mocked"
            }
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Health check failed: {str(e)}")


@app.get("/")
async def root():
    return {"message": "Textbook RAG Mock API is running!"}


# Additional endpoints for compatibility
@app.post("/ingest")
async def ingest_documents():
    """Mock ingest endpoint"""
    return {"status": "success", "documents_processed": 0, "chunks_created": 0}


@app.post("/embed")
async def generate_embedding(text: str):
    """Mock embedding endpoint"""
    return {"embedding": [random.random() for _ in range(1536)], "model": "text-embedding-3-small", "tokens_used": len(text.split())}


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)