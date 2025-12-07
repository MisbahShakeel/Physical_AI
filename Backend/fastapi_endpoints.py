"""
FastAPI Endpoints Specification for Textbook RAG System
"""
from fastapi import FastAPI, HTTPException, Depends, BackgroundTasks
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field
from typing import List, Optional, Dict, Any
import uuid
from datetime import datetime
import asyncio
from contextlib import asynccontextmanager
from openai import OpenAI
from qdrant_client import QdrantClient
import asyncpg
from config import config


# Pydantic models for request/response schemas
class IngestRequest(BaseModel):
    file_paths: List[str]
    force_reprocess: bool = False

class IngestResponse(BaseModel):
    documents_processed: int
    chunks_created: int
    status: str

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

class QuerySelectedRequest(BaseModel):
    query: str = Field(..., min_length=1, max_length=2000)
    selected_text: str = Field(..., min_length=10, max_length=5000)
    session_id: Optional[str] = None
    top_k: int = Field(5, ge=1, le=10)

class FeedbackRequest(BaseModel):
    query_id: str
    rating: int = Field(..., ge=-1, le=1)  # -1: bad, 0: neutral, 1: good
    comment: Optional[str] = Field(None, max_length=1000)

class FeedbackResponse(BaseModel):
    status: str
    query_id: str

class HealthResponse(BaseModel):
    status: str
    timestamp: str
    services: Dict[str, str]

class EmbeddingRequest(BaseModel):
    text: str = Field(..., min_length=1, max_length=5000)
    model: str = "text-embedding-3-small"

class EmbeddingResponse(BaseModel):
    embedding: List[float]
    model: str
    tokens_used: int


# Initialize components
@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup
    # Set the config in app state
    app.state.config = config

    # Initialize clients
    app.state.openai_client = OpenAI(api_key=config.openai_api_key)
    app.state.qdrant_client = QdrantClient(
        url=config.qdrant_url,
        api_key=config.qdrant_api_key,
        timeout=30
    )

    # Initialize PostgreSQL connection pool
    app.state.postgres_pool = await asyncpg.create_pool(
        dsn=config.postgres_dsn,
        min_size=5,
        max_size=20
    )

    # Initialize agent orchestrator (assuming it exists)
    # Note: We'll need to create this class or import it
    try:
        from agent_orchestration import AgentOrchestrator
        app.state.agent_orchestrator = AgentOrchestrator(
            openai_client=app.state.openai_client,
            qdrant_client=app.state.qdrant_client,
            postgres_client=app.state.postgres_pool
        )
    except ImportError:
        print("Warning: AgentOrchestrator not found. Using mock orchestrator.")
        # Create a mock orchestrator for now
        class MockOrchestrator:
            async def process_query(self, query, session_id, selected_text=None):
                return {
                    'response': f"Mock response for query: {query}",
                    'citations': [],
                    'search_mode': 'mock'
                }

        app.state.agent_orchestrator = MockOrchestrator()

    yield

    # Shutdown
    if hasattr(app.state, 'openai_client'):
        app.state.openai_client.close()
    if hasattr(app.state, 'qdrant_client'):
        app.state.qdrant_client.close()
    if hasattr(app.state, 'postgres_pool'):
        await app.state.postgres_pool.close()


# Create FastAPI app
app = FastAPI(
    title="Textbook RAG API",
    description="Retrieval-Augmented Generation API for technical textbook",
    version="1.0.0",
    lifespan=lifespan
)

# CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Configure based on your Docusaurus domain
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# Rate limiting dependency
async def rate_limit_check():
    # Implement rate limiting logic here
    # This could use Redis, in-memory counters, or external service
    pass


# POST /ingest endpoint
@app.post("/ingest", response_model=IngestResponse)
async def ingest_documents(
    request: IngestRequest,
    background_tasks: BackgroundTasks
) -> IngestResponse:
    """
    Ingest documents from Docusaurus markdown files into the RAG system
    """
    start_time = datetime.now()

    try:
        # Initialize ingestor with required clients
        ingestor = TextbookIngestor(
            qdrant_client=app.state.qdrant_client,
            postgres_client=app.state.postgres_pool,
            openai_client=app.state.openai_client
        )

        total_processed = 0
        total_chunks = 0

        for file_path in request.file_paths:
            if request.force_reprocess or not await document_exists(file_path):
                doc_id = await ingestor.ingest_document(file_path)
                chunk_count = await get_chunk_count_for_document(doc_id)
                total_processed += 1
                total_chunks += chunk_count

        processing_time = (datetime.now() - start_time).total_seconds()

        return IngestResponse(
            documents_processed=total_processed,
            chunks_created=total_chunks,
            status="success"
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Ingestion failed: {str(e)}")


# POST /embed endpoint
@app.post("/embed", response_model=EmbeddingResponse)
async def generate_embedding(
    request: EmbeddingRequest
) -> EmbeddingResponse:
    """
    Generate embeddings for text input
    """
    try:
        response = app.state.openai_client.embeddings.create(
            input=[request.text],
            model=request.model
        )

        embedding_data = response.data[0].embedding
        tokens_used = len(request.text.split())  # Approximate token count

        return EmbeddingResponse(
            embedding=embedding_data,
            model=request.model,
            tokens_used=tokens_used
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Embedding generation failed: {str(e)}")


# POST /query endpoint
@app.post("/query", response_model=QueryResponse)
async def query_documents(
    request: QueryRequest,
    rate_limited: Any = Depends(rate_limit_check)
) -> QueryResponse:
    """
    Query the textbook knowledge base with global search
    """
    start_time = datetime.now()

    try:
        # Create or validate session
        session_id = request.session_id or str(uuid.uuid4())
        await create_session_if_not_exists(session_id)

        # Process query through agent orchestrator
        result = await app.state.agent_orchestrator.process_query(
            query=request.query,
            session_id=session_id,
            selected_text=request.selected_text
        )

        latency_ms = int((datetime.now() - start_time).total_seconds() * 1000)

        return QueryResponse(
            response=result['response'],
            citations=result['citations'],
            search_mode=result['search_mode'],
            latency_ms=latency_ms,
            query_id=str(uuid.uuid4())  # Generate query ID for feedback
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Query failed: {str(e)}")


# POST /query-selected endpoint
@app.post("/query-selected", response_model=QueryResponse)
async def query_selected_text(
    request: QuerySelectedRequest,
    rate_limited: Any = Depends(rate_limit_check)
) -> QueryResponse:
    """
    Query specifically within user-selected text
    """
    start_time = datetime.now()

    try:
        session_id = request.session_id or str(uuid.uuid4())
        await create_session_if_not_exists(session_id)

        # Process query with selected-text focus
        result = await app.state.agent_orchestrator.process_query(
            query=request.query,
            session_id=session_id,
            selected_text=request.selected_text
        )

        latency_ms = int((datetime.now() - start_time).total_seconds() * 1000)

        return QueryResponse(
            response=result['response'],
            citations=result['citations'],
            search_mode='selected_text',
            latency_ms=latency_ms,
            query_id=str(uuid.uuid4())
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Selected text query failed: {str(e)}")


# POST /feedback endpoint
@app.post("/feedback", response_model=FeedbackResponse)
async def submit_feedback(
    request: FeedbackRequest
) -> FeedbackResponse:
    """
    Submit feedback on query responses to improve the system
    """
    try:
        await log_feedback({
            'query_id': request.query_id,
            'rating': request.rating,
            'comment': request.comment
        })

        return FeedbackResponse(
            status="Feedback recorded",
            query_id=request.query_id
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Feedback submission failed: {str(e)}")


# GET /stats endpoint
@app.get("/stats")
async def get_system_stats():
    """
    Get system statistics and usage metrics
    """
    try:
        stats = await get_system_statistics()
        return stats
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Stats retrieval failed: {str(e)}")


# Health check endpoint
@app.get("/health", response_model=HealthResponse)
async def health_check():
    """
    Health check endpoint to verify system status
    """
    try:
        # Check connections to all services
        qdrant_status = "connected" if await app.state.qdrant_client.health_check() else "disconnected"

        postgres_status = "connected"
        try:
            async with app.state.postgres_pool.acquire() as conn:
                await conn.fetchval("SELECT 1")
        except:
            postgres_status = "disconnected"

        openai_status = "configured"  # Basic check - API key exists

        return HealthResponse(
            status="healthy",
            timestamp=datetime.now().isoformat(),
            services={
                "qdrant": qdrant_status,
                "postgres": postgres_status,
                "openai": openai_status
            }
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Health check failed: {str(e)}")


# Configuration and utility functions
async def document_exists(file_path: str) -> bool:
    """Check if a document already exists in the system"""
    async with app.state.postgres_pool.acquire() as conn:
        result = await conn.fetchval(
            "SELECT COUNT(*) FROM documents WHERE path = $1",
            file_path
        )
        return result > 0

async def get_chunk_count_for_document(doc_id: str) -> int:
    """Get the number of chunks for a document"""
    async with app.state.postgres_pool.acquire() as conn:
        count = await conn.fetchval(
            "SELECT COUNT(*) FROM chunks WHERE document_id = $1",
            doc_id
        )
        return count

async def create_session_if_not_exists(session_id: str):
    """Create a session if it doesn't exist"""
    async with app.state.postgres_pool.acquire() as conn:
        await conn.execute(
            """
            INSERT INTO sessions (id, created_at, last_activity)
            VALUES ($1, $2, $3)
            ON CONFLICT (id) DO UPDATE SET last_activity = $3
            """,
            session_id, datetime.utcnow(), datetime.utcnow()
        )

async def log_feedback(feedback_data: Dict[str, Any]):
    """Log feedback to the database"""
    async with app.state.postgres_pool.acquire() as conn:
        await conn.execute(
            """
            INSERT INTO feedback (query_id, rating, comment, created_at)
            VALUES ($1, $2, $3, $4)
            """,
            feedback_data['query_id'],
            feedback_data['rating'],
            feedback_data.get('comment'),
            datetime.utcnow()
        )

async def get_system_statistics() -> Dict[str, Any]:
    """Get system usage statistics"""
    async with app.state.postgres_pool.acquire() as conn:
        # Get document count
        doc_count = await conn.fetchval("SELECT COUNT(*) FROM documents")

        # Get chunk count
        chunk_count = await conn.fetchval("SELECT COUNT(*) FROM chunks")

        # Get query count (last 24 hours)
        query_count = await conn.fetchval(
            "SELECT COUNT(*) FROM queries WHERE created_at > $1",
            datetime.utcnow() - timedelta(hours=24)
        )

        # Get average response time (last hour)
        avg_response_time = await conn.fetchval(
            """
            SELECT AVG(latency_ms) FROM queries
            WHERE created_at > $1 AND latency_ms IS NOT NULL
            """,
            datetime.utcnow() - timedelta(hours=1)
        )

        return {
            "documents_count": doc_count,
            "chunks_count": chunk_count,
            "queries_last_24h": query_count,
            "avg_response_time_ms": avg_response_time,
            "timestamp": datetime.utcnow().isoformat()
        }


# Error handlers
@app.exception_handler(404)
async def not_found_handler(request, exc):
    return {"error": "Endpoint not found", "path": str(request.url)}

@app.exception_handler(500)
async def internal_error_handler(request, exc):
    return {"error": "Internal server error", "message": str(exc)}

@app.exception_handler(422)
async def validation_error_handler(request, exc):
    return {"error": "Validation error", "details": exc.errors()}


# Rate limiting endpoints
@app.get("/rate-limit-status")
async def get_rate_limit_status():
    """
    Get current rate limiting status
    """
    # This would return actual rate limit information
    # For now, returning a placeholder
    return {
        "status": "active",
        "requests_per_minute": 100,
        "burst_limit": 10,
        "current_usage": 45
    }


# Utility endpoints for system management
@app.post("/system/clear-cache")
async def clear_system_cache():
    """
    Clear system caches (for maintenance)
    """
    try:
        # This would clear various caches in the system
        # For now, returning a placeholder response
        return {"status": "cache cleared", "timestamp": datetime.utcnow().isoformat()}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Cache clearing failed: {str(e)}")


@app.get("/system/config")
async def get_system_config():
    """
    Get current system configuration
    """
    return {
        "qdrant_collection": "textbook_chunks",
        "embedding_model": "text-embedding-3-small",
        "max_chunk_size": 1000,
        "similarity_threshold": 0.3,
        "top_k_retrieval": 10,
        "temperature": 0.1
    }


# Background tasks
async def process_ingestion_background(file_paths: List[str]):
    """Background task to process document ingestion"""
    ingestor = TextbookIngestor(
        qdrant_client=app.state.qdrant_client,
        postgres_client=app.state.postgres_pool,
        openai_client=app.state.openai_client
    )

    for file_path in file_paths:
        try:
            await ingestor.ingest_document(file_path)
        except Exception as e:
            print(f"Error ingesting {file_path}: {str(e)}")


# Add middleware for request logging
@app.middleware("http")
async def log_requests(request, call_next):
    start_time = datetime.now()
    response = await call_next(request)
    process_time = (datetime.now() - start_time).total_seconds() * 1000

    # Log request info (in a real system, this would go to a proper logging system)
    print(f"{request.method} {request.url.path} - {response.status_code} - {process_time:.2f}ms")

    return response


# Placeholder for database schema initialization
async def init_db():
    """Initialize database tables if they don't exist"""
    async with app.state.postgres_pool.acquire() as conn:
        # Create documents table
        await conn.execute("""
            CREATE TABLE IF NOT EXISTS documents (
                id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
                title VARCHAR(255) NOT NULL,
                author VARCHAR(255),
                version VARCHAR(50),
                path TEXT NOT NULL,
                content_hash VARCHAR(64),
                chunk_count INTEGER,
                created_at TIMESTAMP DEFAULT NOW(),
                updated_at TIMESTAMP DEFAULT NOW()
            )
        """)

        # Create chunks table
        await conn.execute("""
            CREATE TABLE IF NOT EXISTS chunks (
                id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
                document_id UUID REFERENCES documents(id),
                chunk_index INTEGER NOT NULL,
                content TEXT NOT NULL,
                content_type VARCHAR(50) DEFAULT 'text',
                section_title VARCHAR(255),
                chapter_title VARCHAR(255),
                page_number INTEGER,
                section_number VARCHAR(50),
                token_count INTEGER,
                qdrant_point_id VARCHAR(255),
                created_at TIMESTAMP DEFAULT NOW()
            )
        """)

        # Create users table
        await conn.execute("""
            CREATE TABLE IF NOT EXISTS users (
                id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
                email VARCHAR(255) UNIQUE,
                auth_provider VARCHAR(50),
                created_at TIMESTAMP DEFAULT NOW(),
                preferences JSONB
            )
        """)

        # Create sessions table
        await conn.execute("""
            CREATE TABLE IF NOT EXISTS sessions (
                id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
                user_id UUID REFERENCES users(id),
                created_at TIMESTAMP DEFAULT NOW(),
                last_activity TIMESTAMP DEFAULT NOW(),
                metadata JSONB
            )
        """)

        # Create queries table
        await conn.execute("""
            CREATE TABLE IF NOT EXISTS queries (
                id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
                session_id UUID REFERENCES sessions(id),
                user_id UUID REFERENCES users(id),
                query_text TEXT NOT NULL,
                query_type VARCHAR(50),
                selected_text TEXT,
                response_text TEXT,
                citations JSONB,
                latency_ms INTEGER,
                created_at TIMESTAMP DEFAULT NOW()
            )
        """)

        # Create feedback table
        await conn.execute("""
            CREATE TABLE IF NOT EXISTS feedback (
                id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
                query_id UUID REFERENCES queries(id),
                user_id UUID REFERENCES users(id),
                rating INTEGER CHECK (rating >= -1 AND rating <= 1),
                comment TEXT,
                created_at TIMESTAMP DEFAULT NOW()
            )
        """)


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)