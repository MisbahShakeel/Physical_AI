"""
Google Gemini Agent with Qdrant Retrieval Integration

This module implements a Google Gemini Agent that uses a custom retrieval tool
to fetch relevant information from a Qdrant vector database before
answering user queries.
"""

import os
import logging
from typing import Dict, Any
import google.generativeai as genai

from config import config
from retrieval_tool import QdrantRetrievalTool

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class GeminiAgent:
    """
    Google Gemini Agent with custom Qdrant retrieval tool
    """

    def __init__(self, model: str = None):
        # Initialize Google Gemini client
        if not config.GEMINI_API_KEY:
            raise ValueError("GEMINI_API_KEY environment variable is required")

        genai.configure(api_key=config.GEMINI_API_KEY)
        self.model = model or config.GEMINI_MODEL
        self.gemini_client = genai.GenerativeModel(self.model)

        # Initialize retrieval tool
        self.retrieval_tool = QdrantRetrievalTool()

    def query(self, user_query: str, top_k: int = None, similarity_threshold: float = None) -> Dict[str, Any]:
        """
        Process a user query by retrieving relevant context and generating a response

        Args:
            user_query: The user's question or query
            top_k: Number of chunks to retrieve from Qdrant
            similarity_threshold: Minimum similarity score for retrieved chunks

        Returns:
            Dictionary containing the agent's response and supporting information
        """
        # Use config defaults if not provided
        top_k = top_k or config.DEFAULT_TOP_K
        similarity_threshold = similarity_threshold or config.DEFAULT_SIMILARITY_THRESHOLD

        try:
            logger.info(f"Processing query: '{user_query}'")

            # Retrieve relevant context from Qdrant
            retrieved_chunks = self.retrieval_tool.retrieve(
                query=user_query,
                top_k=top_k,
                similarity_threshold=similarity_threshold
            )

            if not retrieved_chunks:
                logger.warning(f"No relevant content found for query: '{user_query}'")
                return {
                    "response": f"I couldn't find specific information about '{user_query}' in the textbook. Please try rephrasing your question or ask about Physical AI and Robotics concepts.",
                    "sources": [],
                    "success": True,
                    "confidence": 0.0
                }

            # Build context from retrieved chunks
            context_parts = []
            for chunk in retrieved_chunks:
                if chunk["content"].strip():
                    context_parts.append(f"Source: {chunk['metadata'].get('title', 'Unknown')}\nContent: {chunk['content']}")

            context = "\n\n".join(context_parts)

            # Create a prompt that includes the retrieved context
            system_message = """You are an expert assistant for the Physical AI and Robotics Learning Framework.
            Answer questions based on the provided context from the textbook.
            Only use information from the context to answer questions.
            If the context doesn't contain information to answer the question, say so clearly.
            Always cite sources when providing information."""

            user_message = f"""Context:\n{context}\n\nQuestion: {user_query}\n\nPlease provide a comprehensive answer based on the context, citing sources where possible."""

            # Combine system message and user message for Gemini
            full_prompt = f"{system_message}\n\n{user_message}"

            # Call Google Gemini API to generate response
            response = self.gemini_client.generate_content(
                full_prompt,
                generation_config=genai.GenerationConfig(
                    temperature=config.DEFAULT_TEMPERATURE,
                    max_output_tokens=config.DEFAULT_MAX_TOKENS
                )
            )

            agent_response = response.text

            logger.info(f"Generated response for query: '{user_query}'")

            return {
                "response": agent_response,
                "sources": retrieved_chunks,
                "success": True,
                "confidence": max([chunk["score"] for chunk in retrieved_chunks]) if retrieved_chunks else 0.0
            }

        except Exception as e:
            logger.error(f"Error processing query '{user_query}': {e}")
            return {
                "response": f"An error occurred while processing your query: {str(e)}",
                "sources": [],
                "success": False,
                "confidence": 0.0
            }


def main():
    """
    Example usage of the Google Gemini Agent with Qdrant retrieval
    """
    try:
        # Initialize the agent
        agent = GeminiAgent()

        # Example query
        query = "What are the key principles of humanoid robotics?"
        print(f"Query: {query}\n")

        # Get response from agent
        result = agent.query(
            user_query=query,
            top_k=config.DEFAULT_TOP_K,
            similarity_threshold=config.DEFAULT_SIMILARITY_THRESHOLD
        )

        print(f"Response: {result['response']}\n")
        print(f"Success: {result['success']}")
        print(f"Confidence: {result['confidence']}")

        if result['sources']:
            print(f"\nSources ({len(result['sources'])} retrieved):")
            for i, source in enumerate(result['sources'][:3], 1):  # Show top 3 sources
                print(f"  {i}. {source['metadata'].get('title', 'Unknown')}")
                print(f"     Score: {source['score']:.3f}")
                print(f"     Content preview: {source['content'][:100]}...")

    except Exception as e:
        logger.error(f"Error in main: {e}")


if __name__ == "__main__":
    main()