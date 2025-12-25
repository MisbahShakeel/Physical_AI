# Data Model: Embedding Pipeline

**Feature**: 1-embedding-pipeline
**Created**: 2025-12-16

## Document Chunk

**Description**: Represents a segment of text extracted from documentation

**Fields**:
- content: string (required) - The text content extracted from the document
- source_url: string (required) - The URL where the content was found
- title: string (required) - The title of the page from which content was extracted
- section: string (optional) - The section or heading under which the content appears
- created_at: datetime (required) - Timestamp when the chunk was created

**Validation Rules**:
- content must not be empty
- source_url must be a valid URL format
- created_at must be in ISO format

## Embedding Vector

**Description**: Numerical representation of text content for similarity search

**Fields**:
- vector: list[float] (required) - The embedding vector from Cohere API
- chunk_id: string (required) - Unique identifier for the source chunk
- payload: dict (required) - Metadata including source_url, title, and other context
- created_at: datetime (required) - Timestamp when the embedding was created

**Validation Rules**:
- vector must have consistent dimensions
- chunk_id must be unique
- payload must include source_url and title

## Metadata

**Description**: Additional information about the original document

**Fields**:
- source_url: string (required) - Original URL of the document
- title: string (required) - Page title
- section: string (optional) - Document section
- processed_at: datetime (required) - When the document was processed
- word_count: integer (optional) - Number of words in the chunk
- token_count: integer (optional) - Number of tokens in the chunk

**Validation Rules**:
- source_url must be valid
- title must not be empty
- word_count and token_count must be non-negative