"""
Qdrant Retrieval Tool for OpenAI Agent

This module implements a custom retrieval tool that connects to Qdrant
for similarity search functionality.
"""

import os
import logging
from typing import Dict, List, Any
from qdrant_client import QdrantClient
import cohere

from config import config

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class QdrantRetrievalTool:
    """
    Custom retrieval tool that connects to Qdrant for similarity search
    """

    def __init__(self):
        # Initialize Cohere client for embeddings
        if not config.COHERE_API_KEY:
            raise ValueError("COHERE_API_KEY environment variable is required")
        self.cohere_client = cohere.Client(config.COHERE_API_KEY)

        # Initialize Qdrant client
        qdrant_client_params = config.get_qdrant_client_params()
        self.qdrant_client = QdrantClient(**qdrant_client_params)

        self.collection_name = config.QDRANT_COLLECTION_NAME

    def retrieve(self, query: str, top_k: int = None, similarity_threshold: float = None) -> List[Dict[str, Any]]:
        """
        Retrieve relevant chunks from Qdrant based on the query

        Args:
            query: The user's query text
            top_k: Number of chunks to retrieve
            similarity_threshold: Minimum similarity score for retrieved chunks

        Returns:
            List of dictionaries containing retrieved content and metadata
        """
        # Use config defaults if not provided
        top_k = top_k or config.DEFAULT_TOP_K
        similarity_threshold = similarity_threshold or config.DEFAULT_SIMILARITY_THRESHOLD

        try:
            # Generate embedding for the query text using Cohere
            response = self.cohere_client.embed(
                texts=[query],
                model=config.COHERE_EMBED_MODEL,
                input_type="search_query"
            )
            query_embedding = response.embeddings[0]

            # Search in Qdrant collection
            search_result = self.qdrant_client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=top_k,
                with_payload=True,
                score_threshold=similarity_threshold
            )

            logger.info(f"Found {len(search_result)} similar results for query: '{query}'")

            # Format results
            retrieved_chunks = []
            for result in search_result:
                chunk = {
                    "content": result.payload.get("content", ""),
                    "score": result.score,
                    "metadata": {
                        "source": result.payload.get("url", ""),
                        "title": result.payload.get("title", ""),
                        "chapter": result.payload.get("chapter", ""),
                        "model": result.payload.get("model", "")
                    }
                }
                retrieved_chunks.append(chunk)

            return retrieved_chunks

        except Exception as e:
            logger.error(f"Retrieval failed: {e}")
            return []

    def get_collection_info(self) -> Any:
        """
        Get information about the Qdrant collection

        Returns:
            Collection information including point count
        """
        try:
            collection_info = self.qdrant_client.get_collection(self.collection_name)
            logger.info(f"Collection '{self.collection_name}' has {collection_info.points_count} points")
            return collection_info
        except Exception as e:
            logger.error(f"Getting collection info failed: {e}")
            return None


# For backward compatibility
def retrieve_from_qdrant(query: str, top_k: int = None, similarity_threshold: float = None) -> List[Dict[str, Any]]:
    """
    Convenience function to retrieve from Qdrant without creating a full agent

    Args:
        query: The user's query text
        top_k: Number of chunks to retrieve
        similarity_threshold: Minimum similarity score for retrieved chunks

    Returns:
        List of dictionaries containing retrieved content and metadata
    """
    tool = QdrantRetrievalTool()
    return tool.retrieve(query, top_k, similarity_threshold)