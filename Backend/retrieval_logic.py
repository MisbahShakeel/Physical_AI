"""
Retrieval and Re-ranking Logic for Textbook RAG System
"""
from typing import List, Dict, Any, Optional
from qdrant_client.http import models
import numpy as np
from openai import AsyncOpenAI
from qdrant_client import AsyncQdrantClient
import asyncio


class TextbookRetriever:
    def __init__(self, qdrant_client: AsyncQdrantClient, postgres_client, openai_client: AsyncOpenAI):
        self.qdrant = qdrant_client
        self.postgres = postgres_client
        self.openai = openai_client

    async def semantic_search(
        self,
        query: str,
        top_k: int = 10,
        selected_text_filter: Optional[str] = None,
        filters: Optional[Dict] = None
    ) -> List[Dict[str, Any]]:
        """Perform semantic search with optional selected-text filtering"""

        # Generate query embedding
        query_embedding = await self.generate_query_embedding(query)

        # Build search filter
        search_filter = self.build_search_filter(selected_text_filter, filters)

        # Perform semantic search
        search_results = await self.qdrant.search(
            collection_name="textbook_chunks",
            query_vector=query_embedding,
            query_filter=search_filter,
            limit=top_k,
            with_payload=True,
            with_vectors=False,
            score_threshold=0.3  # Minimum relevance threshold
        )

        # Extract chunk IDs and scores
        results = []
        for hit in search_results:
            chunk_data = {
                'chunk_id': hit.payload['chunk_id'],
                'document_id': hit.payload['document_id'],
                'content': hit.payload['content'],
                'section_title': hit.payload['section_title'],
                'chapter_title': hit.payload['chapter_title'],
                'relevance_score': hit.score,
                'content_type': hit.payload.get('content_type', 'text')
            }
            results.append(chunk_data)

        # Re-rank using cross-encoder if needed
        if len(results) > 5:
            results = await self.cross_encoder_rerank(query, results)

        return results

    def build_search_filter(
        self,
        selected_text_filter: Optional[str],
        additional_filters: Optional[Dict]
    ) -> Optional[models.Filter]:
        """Build Qdrant search filter based on requirements"""
        conditions = []

        # If selected-text mode, filter to specific content
        if selected_text_filter:
            # This would require pre-processing to identify relevant chunks
            # For now, we'll use a metadata-based approach
            conditions.append(
                models.FieldCondition(
                    key="content",
                    match=models.MatchText(text=selected_text_filter)
                )
            )

        # Add access level filter
        conditions.append(
            models.FieldCondition(
                key="access_level",
                match=models.MatchValue(value="public")
            )
        )

        # Add additional filters
        if additional_filters:
            for key, value in additional_filters.items():
                conditions.append(
                    models.FieldCondition(
                        key=key,
                        match=models.MatchValue(value=value)
                    )
                )

        if conditions:
            return models.Filter(must=conditions)
        return None

    async def cross_encoder_rerank(
        self,
        query: str,
        candidates: List[Dict[str, Any]]
    ) -> List[Dict[str, Any]]:
        """Re-rank top results using cross-encoder for better relevance"""
        # For production, use a cross-encoder model like sentence-transformers
        # For now, we'll implement a simple re-ranking based on query overlap

        query_words = set(query.lower().split())

        for candidate in candidates:
            content_words = set(candidate['content'].lower().split())
            overlap = len(query_words.intersection(content_words))
            candidate['relevance_score'] += overlap * 0.01  # Boost score based on overlap

        # Sort by relevance score
        candidates.sort(key=lambda x: x['relevance_score'], reverse=True)
        return candidates

    async def generate_query_embedding(self, query: str) -> List[float]:
        """Generate embedding for query text"""
        response = await self.openai.embeddings.create(
            input=[query],
            model="text-embedding-3-small"
        )
        return response.data[0].embedding

    def cosine_similarity(self, vec1: List[float], vec2: List[float]) -> float:
        """Calculate cosine similarity between two vectors"""
        dot_product = sum(a * b for a, b in zip(vec1, vec2))
        magnitude1 = sum(a * a for a in vec1) ** 0.5
        magnitude2 = sum(b * b for b in vec2) ** 0.5
        if magnitude1 == 0 or magnitude2 == 0:
            return 0.0
        return dot_product / (magnitude1 * magnitude2)


class SelectedTextRetriever:
    def __init__(self, qdrant_client: AsyncQdrantClient, postgres_client):
        self.qdrant = qdrant_client
        self.postgres = postgres_client

    async def retrieve_from_selected_text(
        self,
        query: str,
        selected_text: str,
        top_k: int = 5
    ) -> List[Dict[str, Any]]:
        """Retrieve relevant chunks only from user-selected text"""

        # First, identify chunks that contain the selected text
        relevant_chunks = await self.find_chunks_containing_text(selected_text)

        if not relevant_chunks:
            # If no direct matches, expand to related chunks
            relevant_chunks = await self.expand_to_related_chunks(selected_text, top_k)

        # Re-rank based on query relevance
        query_embedding = await self.generate_query_embedding_for_text(query)

        scored_chunks = []
        for chunk in relevant_chunks:
            chunk_embedding = await self.get_chunk_embedding(chunk['chunk_id'])
            similarity = self.cosine_similarity(query_embedding, chunk_embedding)

            chunk['relevance_score'] = similarity
            scored_chunks.append(chunk)

        # Sort by relevance and return top-k
        scored_chunks.sort(key=lambda x: x['relevance_score'], reverse=True)
        return scored_chunks[:top_k]

    async def find_chunks_containing_text(self, selected_text: str) -> List[Dict[str, Any]]:
        """Find chunks that contain the selected text"""
        # This could involve:
        # 1. Exact text matching in Qdrant
        # 2. Fuzzy matching
        # 3. Using Postgres full-text search

        # For now, implement using Qdrant's text matching
        search_results = await self.qdrant.search(
            collection_name="textbook_chunks",
            query_filter=models.Filter(
                must=[
                    models.FieldCondition(
                        key="content",
                        match=models.MatchText(text=selected_text[:100])  # Use first 100 chars for matching
                    )
                ]
            ),
            limit=20,
            with_payload=True
        )

        return [
            {
                'chunk_id': hit.payload['chunk_id'],
                'document_id': hit.payload['document_id'],
                'content': hit.payload['content'],
                'section_title': hit.payload['section_title'],
                'chapter_title': hit.payload['chapter_title']
            }
            for hit in search_results
        ]

    async def expand_to_related_chunks(self, selected_text: str, top_k: int) -> List[Dict[str, Any]]:
        """Expand to related chunks when exact text matching fails"""
        # If we can't find exact matches, find semantically related chunks
        # This would use embedding similarity to find related content
        pass

    async def generate_query_embedding_for_text(self, text: str) -> List[float]:
        """Generate embedding for text (query or selected text)"""
        # This would use the same embedding model as the main retriever
        # For now, returning a placeholder
        pass

    async def get_chunk_embedding(self, chunk_id: str) -> List[float]:
        """Retrieve embedding for a specific chunk"""
        # This would fetch the embedding from Qdrant or cache
        # For now, returning a placeholder
        pass

    def cosine_similarity(self, vec1: List[float], vec2: List[float]) -> float:
        """Calculate cosine similarity between two vectors"""
        dot_product = sum(a * b for a, b in zip(vec1, vec2))
        magnitude1 = sum(a * a for a in vec1) ** 0.5
        magnitude2 = sum(b * b for b in vec2) ** 0.5
        if magnitude1 == 0 or magnitude2 == 0:
            return 0.0
        return dot_product / (magnitude1 * magnitude2)


class HybridRetriever:
    """Combines multiple retrieval strategies for better results"""

    def __init__(self, qdrant_client: AsyncQdrantClient, postgres_client, openai_client: AsyncOpenAI):
        self.qdrant = qdrant_client
        self.postgres = postgres_client
        self.openai = openai_client
        self.semantic_retriever = TextbookRetriever(qdrant_client, postgres_client, openai_client)

    async def hybrid_search(
        self,
        query: str,
        top_k: int = 10,
        weights: Dict[str, float] = None
    ) -> List[Dict[str, Any]]:
        """
        Perform hybrid search combining semantic and keyword-based retrieval
        """
        if weights is None:
            weights = {"semantic": 0.7, "keyword": 0.3}

        # Semantic search
        semantic_results = await self.semantic_retriever.semantic_search(query, top_k * 2)

        # Keyword-based search (using Qdrant's keyword search)
        keyword_results = await self.keyword_search(query, top_k * 2)

        # Combine and re-rank
        combined_results = self.combine_results(semantic_results, keyword_results, weights)

        return combined_results[:top_k]

    async def keyword_search(self, query: str, top_k: int) -> List[Dict[str, Any]]:
        """Perform keyword-based search using Qdrant's full-text search"""
        # Split query into keywords
        keywords = query.lower().split()

        # Create filter for keyword matching
        keyword_conditions = []
        for keyword in keywords:
            if len(keyword) > 2:  # Only consider words longer than 2 chars
                keyword_conditions.append(
                    models.FieldCondition(
                        key="content",
                        match=models.MatchText(text=keyword)
                    )
                )

        if not keyword_conditions:
            return []

        # Perform keyword search
        search_results = await self.qdrant.search(
            collection_name="textbook_chunks",
            query_filter=models.Filter(should=keyword_conditions),
            limit=top_k,
            with_payload=True
        )

        # Convert to same format as semantic search
        results = []
        for hit in search_results:
            chunk_data = {
                'chunk_id': hit.payload['chunk_id'],
                'document_id': hit.payload['document_id'],
                'content': hit.payload['content'],
                'section_title': hit.payload['section_title'],
                'chapter_title': hit.payload['chapter_title'],
                'relevance_score': hit.score,
                'content_type': hit.payload.get('content_type', 'text')
            }
            results.append(chunk_data)

        return results

    def combine_results(
        self,
        semantic_results: List[Dict[str, Any]],
        keyword_results: List[Dict[str, Any]],
        weights: Dict[str, float]
    ) -> List[Dict[str, Any]]:
        """
        Combine semantic and keyword results using weighted scoring
        """
        # Create a map of chunk_id to results for deduplication
        result_map = {}

        # Add semantic results with weighted scores
        for result in semantic_results:
            chunk_id = result['chunk_id']
            if chunk_id not in result_map:
                result_map[chunk_id] = result.copy()
            result_map[chunk_id]['semantic_score'] = result['relevance_score'] * weights.get('semantic', 0.7)

        # Add keyword results with weighted scores
        for result in keyword_results:
            chunk_id = result['chunk_id']
            if chunk_id not in result_map:
                result_map[chunk_id] = result.copy()
            result_map[chunk_id]['keyword_score'] = result['relevance_score'] * weights.get('keyword', 0.3)

        # Calculate combined scores
        combined_results = []
        for chunk_id, result in result_map.items():
            semantic_score = result.get('semantic_score', 0)
            keyword_score = result.get('keyword_score', 0)
            combined_score = semantic_score + keyword_score

            result['relevance_score'] = combined_score
            combined_results.append(result)

        # Sort by combined score
        combined_results.sort(key=lambda x: x['relevance_score'], reverse=True)
        return combined_results


class ReRanker:
    """Advanced re-ranking using cross-encoder models"""

    def __init__(self, openai_client: AsyncOpenAI):
        self.openai = openai_client

    async def rerank_with_cross_encoder(
        self,
        query: str,
        candidates: List[Dict[str, Any]]
    ) -> List[Dict[str, Any]]:
        """
        Re-rank candidates using a cross-encoder model
        For this implementation, we'll use OpenAI's model to simulate cross-encoding
        """
        if len(candidates) <= 1:
            return candidates

        # Create pairs of query and candidate content for relevance scoring
        reranked_results = []

        for candidate in candidates:
            relevance_score = await self.calculate_relevance_score(query, candidate['content'])
            candidate['relevance_score'] = relevance_score
            reranked_results.append(candidate)

        # Sort by relevance score
        reranked_results.sort(key=lambda x: x['relevance_score'], reverse=True)
        return reranked_results

    async def calculate_relevance_score(self, query: str, content: str) -> float:
        """
        Calculate relevance score using OpenAI model to simulate cross-encoding
        """
        prompt = f"""
        Rate the relevance of the following text to the query on a scale of 0 to 1.

        Query: {query}

        Text: {content[:1000]}  # Limit to first 1000 chars

        Provide only a number between 0 and 1 representing the relevance score.
        """

        try:
            response = await self.openai.chat.completions.create(
                model="gpt-4o",
                messages=[{"role": "user", "content": prompt}],
                max_tokens=5,
                temperature=0.0
            )

            score_text = response.choices[0].message.content.strip()
            try:
                score = float(score_text)
                return max(0.0, min(1.0, score))  # Clamp between 0 and 1
            except ValueError:
                return 0.5  # Default score if parsing fails
        except Exception:
            return 0.5  # Default score if API call fails


class CacheManager:
    """Manages caching for retrieval operations"""

    def __init__(self):
        self.query_cache = {}
        self.embedding_cache = {}
        self.max_cache_size = 1000

    async def get_cached_results(self, query_hash: str) -> Optional[List[Dict[str, Any]]]:
        """Get cached retrieval results"""
        if query_hash in self.query_cache:
            return self.query_cache[query_hash]
        return None

    async def cache_results(self, query_hash: str, results: List[Dict[str, Any]]):
        """Cache retrieval results"""
        if len(self.query_cache) >= self.max_cache_size:
            # Remove oldest entry
            oldest_key = next(iter(self.query_cache))
            del self.query_cache[oldest_key]

        self.query_cache[query_hash] = results

    async def get_cached_embedding(self, text_hash: str) -> Optional[List[float]]:
        """Get cached embedding"""
        return self.embedding_cache.get(text_hash)

    async def cache_embedding(self, text_hash: str, embedding: List[float]):
        """Cache embedding"""
        if len(self.embedding_cache) >= self.max_cache_size:
            # Remove oldest entry
            oldest_key = next(iter(self.embedding_cache))
            del self.embedding_cache[oldest_key]

        self.embedding_cache[text_hash] = embedding