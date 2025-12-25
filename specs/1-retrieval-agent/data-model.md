# Data Model: Retrieval-Enabled Agent

**Feature**: 1-retrieval-agent
**Date**: 2025-12-17
**Status**: Draft

## Overview
This document defines the data structures and relationships for the retrieval-enabled agent system.

## Core Entities

### 1. Query
Represents a user's input to the agent system.

**Fields**:
- `id` (string, required): Unique identifier for the query
- `text` (string, required): The actual query text from the user
- `timestamp` (datetime, required): When the query was submitted
- `user_id` (string, optional): Identifier for the user (if authenticated)
- `embedding` (array<float>, required): Vector representation of the query text
- `options` (object, optional): Additional query options like top_k, similarity_threshold

**Validation Rules**:
- `text` must be 1-2000 characters
- `embedding` must be a valid float array
- `timestamp` must be in ISO 8601 format

### 2. RetrievedChunk
Represents a piece of content retrieved from the vector database based on similarity to the query.

**Fields**:
- `id` (string, required): Unique identifier for the chunk
- `content` (string, required): The actual text content of the chunk
- `similarity_score` (float, required): Score indicating similarity to the query (0.0-1.0)
- `metadata` (object, required): Additional information about the source
  - `source` (string): Original source document
  - `chapter` (string): Chapter or section identifier
  - `model` (string): Model number (if applicable)
  - `page` (integer, optional): Page number in source
  - `section_title` (string, optional): Title of the section

**Validation Rules**:
- `content` must be 1-10000 characters
- `similarity_score` must be between 0.0 and 1.0
- `metadata.source` is required

### 3. AgentResponse
Represents the agent's response to a user query, including the generated answer and supporting information.

**Fields**:
- `id` (string, required): Unique identifier for the response
- `query_id` (string, required): Reference to the original query
- `response_text` (string, required): The agent's generated response
- `retrieved_context` (array<RetrievedChunk>, required): Chunks used to generate the response
- `confidence_score` (float, optional): Confidence in the response accuracy
- `timestamp` (datetime, required): When the response was generated
- `success` (boolean, required): Whether the operation was successful

**Validation Rules**:
- `response_text` must be 1-10000 characters
- `retrieved_context` must contain 1-20 chunks
- `confidence_score` must be between 0.0 and 1.0 if provided

### 4. QdrantConnectionConfig
Configuration parameters for connecting to the Qdrant vector database.

**Fields**:
- `url` (string, required): The URL of the Qdrant instance
- `api_key` (string, required): API key for authentication
- `collection_name` (string, required): Name of the collection containing embeddings
- `vector_size` (integer, required): Dimension of the embedding vectors
- `distance_metric` (string, required): Distance metric used for similarity search

**Validation Rules**:
- `url` must be a valid URL
- `api_key` must be provided (length > 0)
- `collection_name` must match /^[a-zA-Z][a-zA-Z0-9_-]*$/ pattern

## Relationships

### Query → AgentResponse
- One-to-one relationship
- Each query generates one agent response
- Foreign key: `AgentResponse.query_id` references `Query.id`

### Query → RetrievedChunk (via AgentResponse)
- One-to-many relationship (indirect)
- One query can result in multiple retrieved chunks being used in the response
- Connection: `Query.id` → `AgentResponse.query_id` → `AgentResponse.retrieved_context[]`

## State Transitions

### Query States
- `pending`: Query received, awaiting processing
- `processing`: Agent is retrieving context and generating response
- `completed`: Response generated successfully
- `failed`: Error occurred during processing

### AgentResponse States
- `generating`: Response is being created
- `validated`: Response has been checked for hallucinations
- `ready`: Response is ready for delivery
- `delivered`: Response has been sent to user

## Indexes and Performance Considerations

### Query Entity
- Index on `timestamp` for chronological queries
- Index on `user_id` for user-specific queries (if authentication is implemented)

### RetrievedChunk Entity
- Index on `similarity_score` for sorting by relevance
- Full-text search index on `content` for additional search capabilities

### Performance Guidelines
- Limit retrieved chunks to 5-10 per query for optimal performance
- Cache frequently accessed content chunks
- Implement pagination for large content sets