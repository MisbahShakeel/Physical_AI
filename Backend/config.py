"""Configuration module for the Textbook RAG System"""
import os
from typing import Optional
from pydantic_settings import BaseSettings


class Config(BaseSettings):
    # API Keys and Service Configuration
    openai_api_key: str = os.getenv("OPENAI_API_KEY", "fake-key-for-testing")
    qdrant_url: str = os.getenv("QDRANT_URL", "http://localhost:6333")
    qdrant_api_key: str = os.getenv("QDRANT_API_KEY", "")
    postgres_dsn: str = os.getenv("POSTGRES_DSN", "postgresql://postgres:postgres@localhost/textbook_rag")

    # Application settings
    app_title: str = "Textbook RAG API"
    app_description: str = "Retrieval-Augmented Generation API for technical textbook"
    app_version: str = "1.0.0"

    # Model settings
    embedding_model: str = "text-embedding-3-small"
    max_chunk_size: int = 1000
    similarity_threshold: float = 0.3
    top_k_retrieval: int = 10

    # Rate limiting
    requests_per_minute: int = 100
    burst_limit: int = 10

    class Config:
        env_file = ".env"  # Load variables from .env file if it exists


# Create a singleton config instance
config = Config()