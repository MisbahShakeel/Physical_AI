"""
Qdrant Retrieval Testing Script

This script retrieves data from Qdrant for testing the retrieval pipeline.
It connects to the Qdrant collection and provides functions to search
for similar embeddings and retrieve specific points.
"""

import os
from qdrant_client import QdrantClient
from qdrant_client.http import models
from dotenv import load_dotenv
import logging
import cohere

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Initialize clients
co = cohere.Client(os.getenv("COHERE_API_KEY"))
qdrant_url = os.getenv("QDRANT_URL", "http://localhost:6333")
qdrant_api_key = os.getenv("QDRANT_API_KEY")

if qdrant_api_key:
    qdrant_client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)
else:
    qdrant_client = QdrantClient(url=qdrant_url)

def search_similar(query_text, collection_name="rag_embedding", top_k=10):
    """
    Search for similar embeddings in Qdrant

    Args:
        query_text: Text to find similar embeddings for
        collection_name: Name of the Qdrant collection
        top_k: Number of similar results to return

    Returns:
        List of similar points with payload and score
    """
    try:
        # Generate embedding for the query text using Cohere
        response = co.embed(
            texts=[query_text],
            model="embed-english-v3.0",
            input_type="search_query"  # Using search_query for search queries
        )
        query_embedding = response.embeddings[0]

        # Search in Qdrant collection - using the correct API method
        search_result = qdrant_client.search(
            collection_name=collection_name,
            query_vector=query_embedding,
            limit=top_k,
            with_payload=True
        )

        logger.info(f"Found {len(search_result)} similar results for query: '{query_text}'")
        return search_result
    except AttributeError:
        # Handle older API versions
        logger.warning("Using older Qdrant client API")
        try:
            # Try the legacy search_points method
            search_result = qdrant_client.search_points(
                collection_name=collection_name,
                vector=query_embedding,
                limit=top_k,
                with_payload=True
            )
            logger.info(f"Found {len(search_result.points)} similar results for query: '{query_text}'")
            return search_result.points
        except Exception as e2:
            logger.error(f"Legacy search method failed: {e2}")
            return []
    except Exception as e:
        logger.error(f"Search failed: {e}")
        return []

def get_point_by_id(point_id, collection_name="rag_embedding"):
    """
    Retrieve a specific point by ID from Qdrant

    Args:
        point_id: ID of the point to retrieve
        collection_name: Name of the Qdrant collection

    Returns:
        Point data with payload and vector
    """
    try:
        points = qdrant_client.retrieve(
            collection_name=collection_name,
            ids=[point_id],
            with_payload=True,
            with_vectors=True
        )
        if points:
            logger.info(f"Retrieved point with ID: {point_id}")
            return points[0]
        else:
            logger.warning(f"No point found with ID: {point_id}")
            return None
    except Exception as e:
        logger.error(f"Retrieval by ID failed: {e}")
        return None

def get_collection_info(collection_name="rag_embedding"):
    """
    Get information about the Qdrant collection

    Args:
        collection_name: Name of the Qdrant collection

    Returns:
        Collection information including point count
    """
    try:
        collection_info = qdrant_client.get_collection(collection_name)
        logger.info(f"Collection '{collection_name}' has {collection_info.points_count} points")
        return collection_info
    except Exception as e:
        logger.error(f"Getting collection info failed: {e}")
        return None

def test_retrieval():
    """
    Test function to validate retrieval functionality
    """
    logger.info("Testing Qdrant retrieval...")

    # Get collection info
    info = get_collection_info()
    if info:
        logger.info(f"Collection points count: {info.points_count}")

    # Example search
    query = "Physical AI concepts"
    results = search_similar(query)

    if results:
        logger.info(f"Top result for '{query}':")
        top_result = results[0]
        logger.info(f"  Score: {top_result.score}")
        logger.info(f"  URL: {top_result.payload.get('url', 'N/A')}")
        logger.info(f"  Title: {top_result.payload.get('title', 'N/A')}")
        logger.info(f"  Content preview: {top_result.payload.get('content', 'N/A')[:100]}...")

    # Test retrieval by ID if we have results
    if results:
        sample_id = results[0].id
        point = get_point_by_id(sample_id)
        if point:
            logger.info(f"Successfully retrieved point by ID: {sample_id}")
        else:
            logger.warning(f"Failed to retrieve point by ID: {sample_id}")

    logger.info("Retrieval testing completed")

if __name__ == "__main__":
    test_retrieval()