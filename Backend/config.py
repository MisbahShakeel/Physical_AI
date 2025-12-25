"""
Configuration management for the Google Gemini Agent with Qdrant Retrieval
"""

import os
from typing import Optional
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

class Config:
    """
    Configuration class to manage all settings for the agent system
    """

    # Google Gemini Configuration
    GEMINI_API_KEY: str = os.getenv("GEMINI_API_KEY", "")
    GEMINI_MODEL: str = os.getenv("GEMINI_MODEL", "gemini-2.5-flash")

    # Cohere Configuration (for embeddings)
    COHERE_API_KEY: str = os.getenv("COHERE_API_KEY", "")
    COHERE_EMBED_MODEL: str = os.getenv("COHERE_EMBED_MODEL", "embed-english-v3.0")

    # Qdrant Configuration
    QDRANT_URL: str = os.getenv("QDRANT_URL", "http://localhost:6333")
    QDRANT_API_KEY: Optional[str] = os.getenv("QDRANT_API_KEY")
    QDRANT_COLLECTION_NAME: str = os.getenv("QDRANT_COLLECTION_NAME", "rag_embedding")

    # Agent Configuration
    DEFAULT_TOP_K: int = int(os.getenv("DEFAULT_TOP_K", "5"))
    DEFAULT_SIMILARITY_THRESHOLD: float = float(os.getenv("DEFAULT_SIMILARITY_THRESHOLD", "0.5"))
    DEFAULT_TEMPERATURE: float = float(os.getenv("DEFAULT_TEMPERATURE", "0.3"))
    DEFAULT_MAX_TOKENS: int = int(os.getenv("DEFAULT_MAX_TOKENS", "1000"))

    # Validation
    @classmethod
    def validate(cls) -> bool:
        """
        Validate that all required configuration values are present
        """
        required_vars = [
            cls.GEMINI_API_KEY,
            cls.COHERE_API_KEY,
        ]

        for var in required_vars:
            if not var:
                return False
        return True

    @classmethod
    def get_qdrant_client_params(cls) -> dict:
        """
        Get parameters for initializing Qdrant client
        """
        params = {"url": cls.QDRANT_URL}
        if cls.QDRANT_API_KEY:
            params["api_key"] = cls.QDRANT_API_KEY
        return params

# Initialize config
config = Config()

# Validate configuration on import
if not config.validate():
    print("WARNING: Missing required configuration values. Please check your .env file.")
    print("- GEMINI_API_KEY is required")
    print("- COHERE_API_KEY is required")