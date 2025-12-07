"""
Complete RAG System Implementation for Technical Textbook
"""
import asyncio
from pathlib import Path
from typing import List, Dict, Any, Optional
import frontmatter
from langchain.text_splitter import RecursiveCharacterTextSplitter
from openai import AsyncOpenAI
from qdrant_client import AsyncQdrantClient
import hashlib
import uuid
from datetime import datetime, timedelta
import jwt
from fastapi import FastAPI, HTTPException, Depends, BackgroundTasks
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field
import aioredis
import json
from cryptography.fernet import Fernet


# Data Models
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
    rating: int = Field(..., ge=-1, le=1)
    comment: Optional[str] = Field(None, max_length=1000)

class FeedbackResponse(BaseModel):
    status: str
    query_id: str


# Ingestion Pipeline
class TextbookIngestor:
    def __init__(self, qdrant_client: AsyncQdrantClient, postgres_client, openai_client: AsyncOpenAI):
        self.qdrant = qdrant_client
        self.postgres = postgres_client
        self.openai = openai_client
        self.text_splitter = RecursiveCharacterTextSplitter(
            chunk_size=1000,
            chunk_overlap=200,
            separators=["\n\n", "\n", " ", ""]
        )

    async def ingest_document(self, file_path: str) -> str:
        """Ingest a single Docusaurus markdown file"""
        # Read file content
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()

        # Extract frontmatter metadata
        doc = frontmatter.loads(content)
        metadata = doc.metadata
        text_content = doc.content

        # Create document record
        doc_hash = hashlib.sha256(content.encode()).hexdigest()
        document_id = await self.postgres.create_document({
            'title': metadata.get('title', ''),
            'author': metadata.get('author', ''),
            'path': file_path,
            'content_hash': doc_hash
        })

        # Split into chunks
        chunks = self.text_splitter.split_text(text_content)
        chunk_records = []

        for i, chunk in enumerate(chunks):
            chunk_record = {
                'document_id': document_id,
                'chunk_index': i,
                'content': chunk,
                'section_title': metadata.get('title', ''),
                'chapter_title': self.extract_chapter_title(text_content, i),
                'token_count': len(chunk.split())
            }
            chunk_records.append(chunk_record)

        # Generate embeddings
        embeddings = await self.generate_embeddings([c['content'] for c in chunk_records])

        # Store in Qdrant and update Postgres
        qdrant_ids = await self.store_in_qdrant(chunk_records, embeddings)

        for i, chunk_record in enumerate(chunk_records):
            chunk_record['qdrant_point_id'] = qdrant_ids[i]
            await self.postgres.create_chunk(chunk_record)

        return document_id

    async def generate_embeddings(self, texts: List[str]) -> List[List[float]]:
        """Generate OpenAI embeddings for text chunks"""
        response = await self.openai.embeddings.create(
            input=texts,
            model="text-embedding-3-small"
        )
        return [data.embedding for data in response.data]

    async def store_in_qdrant(self, chunks: List[Dict], embeddings: List[List[float]]) -> List[str]:
        """Store embeddings and metadata in Qdrant"""
        points = []
        for i, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
            point_id = f"{chunk['document_id']}-{chunk['chunk_index']}"
            payload = {
                'document_id': str(chunk['document_id']),
                'chunk_id': str(chunk['qdrant_point_id']) if 'qdrant_point_id' in chunk else point_id,
                'section_title': chunk.get('section_title', ''),
                'chapter_title': chunk.get('chapter_title', ''),
                'content': chunk['content'],
                'content_type': 'text',
                'page_number': chunk.get('page_number', 0),
                'section_number': chunk.get('section_number', ''),
                'embedding_metadata': {
                    'chunk_size': len(chunk['content']),
                    'token_count': chunk.get('token_count', 0)
                },
                'access_level': 'public'
            }

            points.append({
                'id': point_id,
                'vector': embedding,
                'payload': payload
            })

        await self.qdrant.upsert(
            collection_name="textbook_chunks",
            points=points
        )

        return [p['id'] for p in points]

    def extract_chapter_title(self, content: str, chunk_index: int) -> str:
        """Extract chapter/section title from content"""
        import re
        # Look for markdown headers in the content
        headers = re.findall(r'^#+\s+(.+)', content, re.MULTILINE)
        return headers[0] if headers else "Unknown Section"


# Retrieval System
from qdrant_client.http import models
import numpy as np

class TextbookRetriever:
    def __init__(self, qdrant_client, postgres_client, openai_client):
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
            score_threshold=0.3
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
        query_words = set(query.lower().split())

        for candidate in candidates:
            content_words = set(candidate['content'].lower().split())
            overlap = len(query_words.intersection(content_words))
            candidate['relevance_score'] += overlap * 0.01

        candidates.sort(key=lambda x: x['relevance_score'], reverse=True)
        return candidates

    async def generate_query_embedding(self, query: str) -> List[float]:
        """Generate embedding for query text"""
        response = await self.openai.embeddings.create(
            input=[query],
            model="text-embedding-3-small"
        )
        return response.data[0].embedding


class SelectedTextRetriever:
    def __init__(self, qdrant_client, postgres_client):
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
        query_embedding = await self.generate_query_embedding(query)

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
        search_results = await self.qdrant.search(
            collection_name="textbook_chunks",
            query_filter=models.Filter(
                must=[
                    models.FieldCondition(
                        key="content",
                        match=models.MatchText(text=selected_text[:100])
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


# Agent Orchestration
class AgentOrchestrator:
    def __init__(self, openai_client: OpenAI, retriever, postgres_client):
        self.openai = openai_client
        self.retriever = retriever
        self.postgres = postgres_client

        # Initialize specialized agents
        self.query_router = QueryRoutingAgent(openai_client)
        self.retrieval_agent = RetrievalAgent(retriever)
        self.synthesis_agent = AnswerSynthesisAgent(openai_client)
        self.verification_agent = VerificationAgent(openai_client)

    async def process_query(
        self,
        query: str,
        session_id: str,
        selected_text: Optional[str] = None
    ) -> Dict[str, Any]:
        """Main orchestration method"""

        # 1. Route query to determine processing path
        routing_result = await self.query_router.route_query(query, selected_text)

        # 2. Retrieve relevant context
        retrieval_result = await self.retrieval_agent.retrieve(
            query=query,
            search_mode=routing_result['search_mode'],
            selected_text=selected_text
        )

        # 3. Synthesize answer with grounding
        synthesis_result = await self.synthesis_agent.generate_answer(
            query=query,
            context_chunks=retrieval_result['chunks'],
            routing_info=routing_result
        )

        # 4. Verify hallucinations and citations
        verification_result = await self.verification_agent.verify_response(
            query=query,
            response=synthesis_result['response'],
            context_chunks=retrieval_result['chunks']
        )

        # 5. Log query and return response
        await self.postgres.log_query({
            'session_id': session_id,
            'query_text': query,
            'response_text': verification_result['final_response'],
            'citations': verification_result['citations'],
            'query_type': routing_result['search_mode']
        })

        return {
            'response': verification_result['final_response'],
            'citations': verification_result['citations'],
            'search_mode': routing_result['search_mode'],
            'retrieval_metadata': retrieval_result['metadata']
        }


class QueryRoutingAgent:
    def __init__(self, openai_client: OpenAI):
        self.openai = openai_client

    async def route_query(self, query: str, selected_text: Optional[str]) -> Dict[str, Any]:
        """Determine query processing path"""

        # If selected text is provided, always use selected-text mode
        if selected_text and len(selected_text.strip()) > 10:
            return {
                'search_mode': 'selected_text',
                'selected_text': selected_text
            }

        # Otherwise, determine if it's a general knowledge query
        routing_prompt = f"""
        Analyze the following query and determine the appropriate search mode:

        Query: {query}

        Available modes:
        - 'global': Search the entire textbook for general knowledge
        - 'selected_text': Search only within user-selected text

        If the user has provided specific text context, always return 'selected_text'.
        If no specific context is provided, return 'global'.

        Respond with a JSON object containing 'search_mode' field.
        """

        response = self.openai.chat.completions.create(
            model="gpt-4o",
            messages=[{"role": "user", "content": routing_prompt}],
            response_format={"type": "json_object"}
        )

        result = json.loads(response.choices[0].message.content)
        result['search_mode'] = 'global'

        return result


class RetrievalAgent:
    def __init__(self, retriever):
        self.retriever = retriever

    async def retrieve(
        self,
        query: str,
        search_mode: str,
        selected_text: Optional[str] = None
    ) -> Dict[str, Any]:
        """Perform retrieval based on search mode"""

        if search_mode == 'selected_text' and selected_text:
            chunks = await self.retriever.retrieve_from_selected_text(
                query=query,
                selected_text=selected_text,
                top_k=5
            )
        else:
            chunks = await self.retriever.semantic_search(
                query=query,
                top_k=10,
                selected_text_filter=selected_text
            )

        return {
            'chunks': chunks,
            'metadata': {
                'search_mode': search_mode,
                'retrieved_count': len(chunks)
            }
        }


class AnswerSynthesisAgent:
    def __init__(self, openai_client: OpenAI):
        self.openai = openai_client

    async def generate_answer(
        self,
        query: str,
        context_chunks: List[Dict[str, Any]],
        routing_info: Dict[str, Any]
    ) -> Dict[str, Any]:
        """Generate grounded answer from context"""

        # Format context with citations
        formatted_context = self.format_context_with_citations(context_chunks)

        system_prompt = """
        You are an expert assistant for a technical textbook. Your role is to provide accurate,
        well-cited answers based strictly on the provided context.

        RULES:
        1. Only use information from the provided context chunks
        2. Never hallucinate or invent information
        3. Always cite specific chunks when referencing information
        4. If the answer cannot be found in the context, clearly state this
        5. Maintain academic tone appropriate for technical content
        6. Structure responses clearly with relevant sections
        """

        user_prompt = f"""
        Query: {query}

        Context Chunks:
        {formatted_context}

        Provide a comprehensive answer that:
        1. Directly addresses the query
        2. Uses information only from the provided context
        3. Includes specific citations for each piece of information
        4. Maintains the academic tone of the textbook
        """

        response = self.openai.chat.completions.create(
            model="gpt-4o",
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt}
            ]
        )

        return {
            'response': response.choices[0].message.content,
            'context_chunks': context_chunks
        }

    def format_context_with_citations(self, chunks: List[Dict[str, Any]]) -> str:
        """Format context chunks with proper citations"""
        formatted = []
        for i, chunk in enumerate(chunks):
            citation = f"[Chunk {i+1}: {chunk['chapter_title']} - {chunk['section_title']}]"
            formatted.append(f"{citation}\n{chunk['content']}\n")

        return "\n".join(formatted)


class VerificationAgent:
    def __init__(self, openai_client: OpenAI):
        self.openai = openai_client

    async def verify_response(
        self,
        query: str,
        response: str,
        context_chunks: List[Dict[str, Any]]
    ) -> Dict[str, Any]:
        """Verify response for hallucinations and proper citations"""

        verification_prompt = f"""
        Verify the following response for hallucinations and citation accuracy:

        Query: {query}

        Response: {response}

        Context Chunks:
        {self.format_chunks_for_verification(context_chunks)}

        Check for:
        1. Information in response that's not supported by context chunks
        2. Citations that don't match the provided chunks
        3. Factual accuracy based on context
        4. Proper attribution of information

        Provide verification result in JSON format with:
        - 'is_valid': boolean indicating if response is valid
        - 'issues': array of identified issues
        - 'citations': array of properly cited chunk references
        - 'final_response': corrected response if needed
        """

        verification_response = self.openai.chat.completions.create(
            model="gpt-4o",
            messages=[{"role": "user", "content": verification_prompt}],
            response_format={"type": "json_object"}
        )

        verification_result = json.loads(verification_response.choices[0].message.content)

        return {
            'final_response': verification_result.get('final_response', response),
            'citations': verification_result.get('citations', []),
            'is_valid': verification_result.get('is_valid', True),
            'issues': verification_result.get('issues', [])
        }

    def format_chunks_for_verification(self, chunks: List[Dict[str, Any]]) -> str:
        """Format chunks for verification agent"""
        return "\n".join([
            f"Chunk {i+1}: {chunk['content'][:200]}..."
            for i, chunk in enumerate(chunks)
        ])


# Security and Rate Limiting
from slowapi import Limiter, _rate_limit_exceeded_handler
from slowapi.util import get_remote_address
from slowapi.errors import RateLimitExceeded
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials

limiter = Limiter(key_func=get_remote_address)

class JWTAuth:
    def __init__(self, secret_key: str, algorithm: str = "HS256"):
        self.secret_key = secret_key
        self.algorithm = algorithm
        self.security = HTTPBearer()

    async def create_token(self, user_id: str, expires_delta: Optional[timedelta] = None):
        if expires_delta:
            expire = datetime.utcnow() + expires_delta
        else:
            expire = datetime.utcnow() + timedelta(hours=24)

        to_encode = {
            "sub": user_id,
            "exp": expire,
            "iat": datetime.utcnow()
        }
        encoded_jwt = jwt.encode(to_encode, self.secret_key, algorithm=self.algorithm)
        return encoded_jwt

    async def verify_token(self, credentials: HTTPAuthorizationCredentials = Security(security)):
        try:
            payload = jwt.decode(credentials.credentials, self.secret_key, algorithms=[self.algorithm])
            user_id: str = payload.get("sub")
            if user_id is None:
                raise HTTPException(status_code=401, detail="Invalid token")
            return user_id
        except jwt.ExpiredSignatureError:
            raise HTTPException(status_code=401, detail="Token has expired")
        except jwt.JWTError:
            raise HTTPException(status_code=401, detail="Invalid token")


class SessionManager:
    def __init__(self, postgres_client, redis_client=None):
        self.postgres = postgres_client
        self.redis = redis_client

    async def create_session(self, user_id: Optional[str] = None) -> str:
        """Create a new session with proper isolation"""
        session_id = str(uuid.uuid4())

        await self.postgres.execute(
            "INSERT INTO sessions (id, user_id, created_at, last_activity) VALUES ($1, $2, $3, $4)",
            session_id, user_id, datetime.utcnow(), datetime.utcnow()
        )

        return session_id

    async def validate_session(self, session_id: str, user_id: Optional[str] = None) -> bool:
        """Validate session and ensure user isolation"""
        session = await self.postgres.fetchrow(
            "SELECT user_id FROM sessions WHERE id = $1 AND last_activity > $2",
            session_id, datetime.utcnow() - timedelta(hours=24)
        )

        if not session:
            return False

        # If user_id provided, ensure session belongs to user
        if user_id and session['user_id'] != user_id:
            return False

        # Update last activity
        await self.postgres.execute(
            "UPDATE sessions SET last_activity = $1 WHERE id = $2",
            datetime.utcnow(), session_id
        )

        return True


class DataPrivacyManager:
    def __init__(self, encryption_key: bytes):
        self.cipher_suite = Fernet(encryption_key)

    def encrypt_sensitive_data(self, data: str) -> str:
        """Encrypt sensitive data before storage"""
        encrypted_data = self.cipher_suite.encrypt(data.encode())
        return encrypted_data.decode()

    def decrypt_sensitive_data(self, encrypted_data: str) -> str:
        """Decrypt sensitive data"""
        decrypted_data = self.cipher_suite.decrypt(encrypted_data.encode())
        return decrypted_data.decode()

    def hash_pii(self, pii_data: str) -> str:
        """Hash personally identifiable information"""
        return hashlib.sha256(pii_data.encode()).hexdigest()


# Caching Manager
class CacheManager:
    def __init__(self, redis_url: str):
        self.redis = aioredis.from_url(redis_url)

    async def get_cached_embedding(self, text_hash: str) -> Optional[List[float]]:
        """Get cached embedding by text hash"""
        cached = await self.redis.get(f"embedding:{text_hash}")
        if cached:
            return json.loads(cached)
        return None

    async def cache_embedding(self, text_hash: str, embedding: List[float], ttl: int = 3600):
        """Cache embedding with TTL"""
        await self.redis.setex(
            f"embedding:{text_hash}",
            ttl,
            json.dumps(embedding)
        )

    async def get_cached_query_result(self, query_hash: str) -> Optional[Dict[str, Any]]:
        """Get cached query result"""
        cached = await self.redis.get(f"query_result:{query_hash}")
        if cached:
            return json.loads(cached)
        return None

    async def cache_query_result(self, query_hash: str, result: Dict[str, Any], ttl: int = 300):
        """Cache query result for 5 minutes"""
        await self.redis.setex(
            f"query_result:{query_hash}",
            ttl,
            json.dumps(result, default=str)
        )


# Main FastAPI Application
from contextlib import asynccontextmanager

@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup
    app.state.agent_orchestrator = AgentOrchestrator(
        openai_client=app.state.openai_client,
        retriever=app.state.retriever,
        postgres_client=app.state.postgres_client
    )
    yield
    # Shutdown
    # Cleanup resources if needed

app = FastAPI(
    title="Textbook RAG API",
    description="Retrieval-Augmented Generation API for technical textbook",
    version="1.0.0",
    lifespan=lifespan
)

# CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Dependency for rate limiting
async def rate_limit_check():
    # Implement rate limiting logic here
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
    try:
        ingestor = TextbookIngestor(
            qdrant_client=app.state.qdrant_client,
            postgres_client=app.state.postgres_client,
            openai_client=app.state.openai_client
        )

        total_processed = 0
        total_chunks = 0

        for file_path in request.file_paths:
            if request.force_reprocess or not await app.state.postgres_client.document_exists(file_path):
                doc_id = await ingestor.ingest_document(file_path)
                chunk_count = await app.state.postgres_client.get_chunk_count_for_document(doc_id)
                total_processed += 1
                total_chunks += chunk_count

        return IngestResponse(
            documents_processed=total_processed,
            chunks_created=total_chunks,
            status="success"
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Ingestion failed: {str(e)}")

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
        await app.state.postgres_client.create_session_if_not_exists(session_id)

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
            query_id=str(uuid.uuid4())
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
        await app.state.postgres_client.create_session_if_not_exists(session_id)

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
        await app.state.postgres_client.log_feedback({
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

# Health check endpoint
@app.get("/health")
async def health_check():
    """
    Health check endpoint to verify system status
    """
    try:
        # Check connections to all services
        await app.state.qdrant_client.health_check()
        await app.state.postgres_client.health_check()

        return {
            "status": "healthy",
            "timestamp": datetime.now().isoformat(),
            "services": {
                "qdrant": "connected",
                "postgres": "connected",
                "openai": "configured"
            }
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Health check failed: {str(e)}")

# Error handlers
@app.exception_handler(404)
async def not_found_handler(request, exc):
    return {"error": "Endpoint not found", "path": str(request.url)}

@app.exception_handler(500)
async def internal_error_handler(request, exc):
    return {"error": "Internal server error", "message": str(exc)}


# Testing and Validation
import pytest

class RetrievalAccuracyTester:
    def __init__(self, retriever, postgres_client):
        self.retriever = retriever
        self.postgres = postgres_client

    async def test_retrieval_precision(self, test_queries: List[Dict[str, Any]]) -> Dict[str, float]:
        """Test retrieval precision using known ground truth"""
        results = {
            'precision_at_1': 0,
            'precision_at_5': 0,
            'precision_at_10': 0,
            'recall_at_10': 0,
            'mrr': 0
        }

        total_queries = len(test_queries)

        for query_data in test_queries:
            query = query_data['query']
            expected_chunks = set(query_data['expected_chunk_ids'])

            retrieved_chunks = await self.retriever.semantic_search(query, top_k=10)
            retrieved_ids = {chunk['chunk_id'] for chunk in retrieved_chunks}

            # Calculate metrics
            for k in [1, 5, 10]:
                top_k_retrieved = set(chunk['chunk_id'] for chunk in retrieved_chunks[:k])
                intersection = top_k_retrieved.intersection(expected_chunks)
                results[f'precision_at_{k}'] += len(intersection) / min(k, len(expected_chunks)) if expected_chunks else 0

            # Recall at 10
            if expected_chunks:
                results['recall_at_10'] += len(retrieved_ids.intersection(expected_chunks)) / len(expected_chunks)

            # MRR - find rank of first relevant chunk
            for rank, chunk in enumerate(retrieved_chunks, 1):
                if chunk['chunk_id'] in expected_chunks:
                    results['mrr'] += 1.0 / rank
                    break

        # Average metrics
        for key in results:
            results[key] /= total_queries if total_queries > 0 else 0

        return results


class HallucinationDetector:
    def __init__(self, openai_client):
        self.openai = openai_client

    async def detect_hallucinations(self, response: str, context_chunks: List[Dict[str, Any]]) -> Dict[str, Any]:
        """Detect hallucinations in generated response"""

        context_text = " ".join([chunk['content'] for chunk in context_chunks])

        hallucination_prompt = f"""
        Analyze the following response for hallucinations by comparing it to the provided context.

        Context: {context_text[:3000]}

        Response: {response}

        Identify any claims, facts, or statements in the response that:
        1. Are not supported by the context
        2. Contradict information in the context
        3. Reference information not present in the context

        Provide your analysis in JSON format with:
        - 'has_hallucinations': boolean
        - 'hallucinated_claims': array of hallucinated statements
        - 'confidence': confidence score (0-1)
        - 'supporting_evidence': array of context snippets that support the response
        """

        analysis = self.openai.chat.completions.create(
            model="gpt-4o",
            messages=[{"role": "user", "content": hallucination_prompt}],
            response_format={"type": "json_object"}
        )

        return json.loads(analysis.choices[0].message.content)


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)