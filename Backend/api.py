"""
FastAPI backend for the Physical AI and Robotics Learning Framework
"""

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import Optional, List
import logging
import sys
import os

# Add the backend directory to the path to import retrieve module
sys.path.append(os.path.join(os.path.dirname(__file__)))

import google.generativeai as genai
from retrieve import search_similar
from agent import GeminiAgent
from config import config

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = FastAPI(title="Physical AI and Robotics Learning Framework API",
              description="Backend API for textbook Q&A and content retrieval",
              version="1.0.0")

# Add CORS middleware to allow requests from frontend
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

class QueryRequest(BaseModel):
    query: str
    top_k: Optional[int] = 10  # Increased default to return more results
    collection_name: Optional[str] = "rag_embedding"

class Citation(BaseModel):
    chapter_title: str
    section_title: str
    page_number: Optional[int] = None
    module: Optional[str] = None
    url: Optional[str] = None
    content_preview: Optional[str] = None
    score: Optional[float] = None

class QueryResponse(BaseModel):
    response: str
    citations: List[Citation]
    search_mode: str = "global"
    latency_ms: int
    query_id: str

@app.get("/health")
async def health_check():
    """Health check endpoint to verify the system is running."""
    return {"status": "healthy", "service": "Physical AI Backend"}

@app.post("/query", response_model=QueryResponse)
async def query_endpoint(request: QueryRequest):
    """Main endpoint for textbook Q&A based on vector search and optional LLM generation."""
    try:
        logger.info(f"Processing query: '{request.query}' with top_k: {request.top_k}")

        # Perform similarity search in the vector database
        search_results = search_similar(
            query_text=request.query,
            collection_name=request.collection_name,
            top_k=request.top_k  # Use the requested number of results
        )

        logger.info(f"Found {len(search_results)} results for query: '{request.query}'")

        # Build response content from the search results
        if search_results:
            # Combine content from top results to create a comprehensive response
            combined_content = []
            citations = []

            for result in search_results:
                if hasattr(result, 'payload') and result.payload:
                    content = result.payload.get('content', '')
                    if content:
                        combined_content.append(content)

                    # Create citation from result
                    citation = Citation(
                        chapter_title=result.payload.get('title', 'Untitled'),
                        section_title=result.payload.get('url', ''),
                        content_preview=content[:200] if content else '',
                        score=float(getattr(result, 'score', 0)),
                        url=result.payload.get('url', '')
                    )
                    citations.append(citation)

                # Limit the response content to avoid overly long responses
                if len(combined_content) >= min(5, len(search_results)):  # Take max 5 chunks
                    break

            # Check if we have the required API keys to use the Gemini agent
            if config.GEMINI_API_KEY and config.COHERE_API_KEY:
                try:
                    # Initialize the Gemini agent
                    agent = GeminiAgent()

                    # Format the retrieved context for the agent
                    context_parts = []
                    for chunk in search_results[:3]:  # Use top 3 results
                        if hasattr(chunk, 'payload') and chunk.payload:
                            content = chunk.payload.get('content', '')
                            if content.strip():
                                title = chunk.payload.get('title', 'Unknown')
                                context_parts.append(f"Source: {title}\nContent: {content}")

                    context = "\n\n".join(context_parts)

                    # Generate a more sophisticated response using the Gemini agent
                    system_message = """You are an expert assistant for the Physical AI and Robotics Learning Framework.
                    Answer questions based on the provided context from the textbook.
                    Only use information from the context to answer questions.
                    If the context doesn't contain information to answer the question, say so clearly.
                    Always cite sources when providing information."""

                    user_message = f"""Context:\n{context}\n\nQuestion: {request.query}\n\nPlease provide a comprehensive answer based on the context, citing sources where possible."""

                    # Combine system message and user message for Gemini
                    full_prompt = f"{system_message}\n\n{user_message}"

                    # Call Google Gemini API to generate response
                    gemini_response = agent.gemini_client.generate_content(
                        full_prompt,
                        generation_config=genai.GenerationConfig(
                            temperature=config.DEFAULT_TEMPERATURE,
                            max_output_tokens=config.DEFAULT_MAX_TOKENS
                        )
                    )

                    response_text = gemini_response.text
                    search_mode = "llm_enhanced_search"

                except Exception as agent_error:
                    logger.warning(f"Gemini agent failed, falling back to basic search: {agent_error}")
                    # Fallback to the original approach
                    response_text = "Based on the textbook content, here's what I found: "
                    response_text += " ".join(combined_content[:3])  # Use first 3 results for response
                    search_mode = "vector_search"
            else:
                # Use the original approach when API keys are not available
                response_text = "Based on the textbook content, here's what I found: "
                response_text += " ".join(combined_content[:3])  # Use first 3 results for response
                search_mode = "vector_search"

            if not response_text or len(response_text.strip()) < 20:
                response_text = f"I found {len(search_results)} relevant sections in the textbook related to '{request.query}'. For more details, please refer to the cited sources."
        else:
            response_text = f"I couldn't find specific information about '{request.query}' in the textbook. Please try rephrasing your question or ask about Physical AI and Robotics concepts."
            citations = []
            search_mode = "vector_search"

        # Create response
        response = QueryResponse(
            response=response_text,
            citations=citations,
            search_mode=search_mode,
            latency_ms=100,  # Placeholder - would be actual processing time
            query_id=f"query-{hash(request.query) % 1000000}"
        )

        return response

    except Exception as e:
        logger.error(f"Error processing query '{request.query}': {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error processing query: {str(e)}")

@app.get("/")
async def root():
    """Root endpoint providing API information."""
    return {
        "message": "Physical AI and Robotics Learning Framework API",
        "endpoints": {
            "POST /query": "Submit a query to search the textbook content",
            "GET /health": "Check the health status of the service",
            "GET /config": "Get API configuration and capabilities"
        }
    }

@app.get("/config")
async def get_config():
    """Get API configuration and capabilities."""
    return {
        "model": config.GEMINI_MODEL,
        "has_gemini_api": bool(config.GEMINI_API_KEY),
        "has_cohere_api": bool(config.COHERE_API_KEY),
        "has_qdrant_connection": bool(config.QDRANT_URL),
        "default_top_k": config.DEFAULT_TOP_K,
        "default_temperature": config.DEFAULT_TEMPERATURE,
        "default_max_tokens": config.DEFAULT_MAX_TOKENS,
        "qdrant_collection": config.QDRANT_COLLECTION_NAME
    }

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)