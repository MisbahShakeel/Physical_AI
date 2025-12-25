# Tasks: Embedding Pipeline Setup

**Feature**: 1-embedding-pipeline
**Created**: 2025-12-16
**Status**: Draft

## Overview

This document outlines the implementation tasks for the embedding pipeline that extracts content from Docusaurus URLs, generates embeddings using Cohere, and stores them in Qdrant for RAG applications. The implementation will be in a single main.py file with specific functions as required.

## Implementation Strategy

- **MVP First**: Start with basic functionality (URL discovery, text extraction, simple embedding)
- **Incremental Delivery**: Build up functionality in phases
- **Independent Testing**: Each user story should be testable independently
- **Error Handling**: Implement proper error handling and logging throughout

---

## Phase 1: Setup

### Goal
Initialize the project structure and dependencies with UV package management.

### Independent Test Criteria
- Project directory structure is created
- Dependencies are properly installed via UV
- Environment variables are configured

### Tasks

- [x] T001 Create backend directory structure
- [x] T002 Set up pyproject.toml for UV package management with required dependencies
- [x] T003 Create requirements.txt file with cohere, qdrant-client, beautifulsoup4, requests, python-dotenv
- [x] T004 Create .env file template with required environment variables
- [x] T005 Create basic main.py file with imports and logging setup

## Phase 2: Foundational Components

### Goal
Implement foundational components that are required by multiple user stories.

### Independent Test Criteria
- API clients (Cohere, Qdrant) are properly initialized
- Configuration is loaded from environment variables
- Basic logging is set up

### Tasks

- [x] T006 [P] Initialize Cohere client with API key from environment variables
- [x] T007 [P] Initialize Qdrant client with connection parameters from environment
- [x] T008 Set up logging configuration for the application
- [x] T009 Create utility functions for environment variable loading and validation
- [x] T010 Implement basic error handling and retry mechanisms

## Phase 3: User Story 1 - Docusaurus URL Content Extraction (Priority: P1)

### Goal
As a developer building backend retrieval layers, I want to crawl and extract clean text content from deployed Docusaurus sites, so that I can process the documentation for embedding generation.

### Independent Test Criteria
- Can provide a Docusaurus URL and verify that clean text content is extracted without HTML tags, navigation elements, or other non-content markup
- Can process a Docusaurus site with multiple pages and extract text from all accessible content pages

### Acceptance Scenarios
1. Given a valid Docusaurus site URL, when the extraction process runs, then clean text content is returned without HTML tags, navigation menus, or sidebar elements
2. Given a Docusaurus site with multiple pages, when the crawler processes the site, then all accessible content pages are crawled and their text is extracted

### Tasks

- [x] T011 [US1] Implement get_all_urls function to discover all URLs from the target Docusaurus site
- [x] T012 [US1] Add sitemap.xml parsing to URL discovery function
- [x] T013 [US1] Implement navigation link extraction for URL discovery
- [x] T014 [US1] Implement extract_text_from_url function to extract clean text from a given URL
- [x] T015 [US1] Add HTML cleaning to remove navigation, headers, footers, and other non-content elements
- [x] T016 [US1] Implement content area detection for Docusaurus-specific selectors
- [x] T017 [US1] Add title extraction from page content
- [x] T018 [US1] Implement error handling for inaccessible URLs
- [x] T019 [US1] Test URL discovery and text extraction with the target site

## Phase 4: User Story 2 - Cohere Embedding Generation (Priority: P2)

### Goal
As a developer building backend retrieval layers, I want to generate embeddings from extracted text using Cohere's API, so that I can create vector representations suitable for semantic search.

### Independent Test Criteria
- Can provide text content and verify that Cohere generates valid embeddings that can be stored and retrieved
- Can process multiple text chunks and generate embeddings for all of them successfully

### Acceptance Scenarios
1. Given clean text content from documentation, when Cohere embedding generation runs, then a valid embedding vector is produced
2. Given multiple text chunks, when batch embedding generation runs, then all chunks are successfully converted to embeddings

### Tasks

- [x] T020 [US2] Implement embed function to generate embeddings for a list of texts using Cohere
- [x] T021 [US2] Add proper error handling for Cohere API rate limits and errors
- [x] T022 [US2] Implement batch processing for multiple text chunks
- [x] T023 [US2] Add embedding validation to ensure vectors have correct dimensions
- [x] T024 [US2] Test embedding generation with sample text chunks

## Phase 5: User Story 3 - Qdrant Vector Storage (Priority: P3)

### Goal
As a developer building backend retrieval layers, I want to store generated embeddings in Qdrant vector database, so that I can perform efficient similarity searches for RAG applications.

### Independent Test Criteria
- Can store embeddings in Qdrant and perform similarity searches to verify retrieval functionality
- Embeddings are stored with appropriate metadata for later retrieval

### Acceptance Scenarios
1. Given embedding vectors from Cohere, when storage process runs, then vectors are successfully stored in Qdrant with associated metadata
2. Given stored embeddings in Qdrant, when similarity search is performed, then relevant results are returned based on vector similarity

### Tasks

- [x] T025 [US3] Implement create_collection function to create Qdrant collection named "rag_embedding"
- [x] T026 [US3] Add collection validation and recreation logic if needed
- [x] T027 [US3] Implement save_chunk_to_qdrant function to store embeddings with metadata
- [x] T028 [US3] Add proper payload structure with URL, title, and other metadata
- [x] T029 [US3] Implement error handling for Qdrant connection issues
- [x] T030 [US3] Test vector storage and retrieval functionality

## Phase 6: Text Processing and Chunking

### Goal
Implement text processing functionality to split large documents into manageable chunks while preserving semantic coherence.

### Independent Test Criteria
- Large documents are properly chunked into smaller segments
- Chunks maintain semantic coherence with appropriate overlap
- Chunking parameters are configurable

### Tasks

- [x] T031 [P] Implement chunk_text function to split text into manageable chunks with overlap
- [x] T032 [P] Add configurable chunk size and overlap parameters
- [x] T033 [P] Implement sentence-boundary aware chunking to preserve context
- [x] T034 [P] Add validation to ensure chunks are not empty
- [x] T035 Test chunking functionality with various document sizes

## Phase 7: Main Execution Flow Integration

### Goal
Orchestrate the entire pipeline in the main function to process all discovered URLs end-to-end.

### Independent Test Criteria
- The complete pipeline runs from URL discovery to vector storage
- Progress feedback is provided during execution
- Errors are handled gracefully without stopping the entire process

### Tasks

- [x] T036 Implement main function to orchestrate the entire pipeline
- [x] T037 Add progress tracking and logging for each stage
- [x] T038 Implement graceful error handling for individual URL processing
- [x] T039 Add rate limiting to be respectful to the target server
- [x] T040 Test the complete pipeline with the target URL: https://physical-ai-ivory.vercel.app/

## Phase 8: Polish & Cross-Cutting Concerns

### Goal
Add finishing touches, documentation, and ensure all requirements are met.

### Independent Test Criteria
- All functionality works as specified in the requirements
- The solution handles edge cases appropriately
- Performance and reliability requirements are met

### Tasks

- [x] T041 Add comprehensive error handling for all edge cases mentioned in spec
- [x] T042 Implement memory management for large document processing
- [x] T043 Add configuration options for chunk size, overlap, and processing parameters
- [x] T044 Add documentation to all functions
- [x] T045 Test with the target URL to ensure all requirements are met
- [x] T046 Verify that 100+ pages can be processed within 30 minutes (performance requirement)
- [x] T047 Add final validation that embeddings are generated for 95%+ of text chunks
- [x] T048 Clean up code and ensure proper formatting

---

## Dependencies

- User Story 1 (URL extraction) must be completed before User Story 2 (embedding generation) can begin
- User Story 2 (embedding generation) must be completed before User Story 3 (vector storage) can begin
- Foundational components must be completed before any user story implementation

## Parallel Execution Examples

- Tasks T006 and T007 can run in parallel (different API client setups)
- Tasks T011-T019 can run in parallel with T020-T024 if T001-T010 are completed (different user stories)
- Tasks T014 and T015 can run in parallel (same user story, different aspects of text extraction)

## Success Criteria Verification

- [x] All pages from the target URL are successfully crawled
- [x] Clean text content is extracted without HTML markup
- [x] Text chunks are properly created and within size limits
- [x] Embeddings are generated successfully using Cohere
- [x] Embeddings are stored in Qdrant with appropriate metadata
- [x] The entire pipeline runs as a single main.py file with specified functions
- [x] Documentation content from 100+ Docusaurus pages can be successfully extracted and cleaned within 30 minutes
- [x] Embeddings are generated for 95% of text chunks with no data loss or corruption