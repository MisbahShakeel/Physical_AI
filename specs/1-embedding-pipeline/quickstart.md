# Quickstart Guide: Embedding Pipeline

**Feature**: 1-embedding-pipeline
**Created**: 2025-12-16

## Prerequisites

- Python 3.8 or higher
- UV package manager
- Cohere API key
- Qdrant instance (local or remote)

## Setup

1. **Clone the repository**
   ```bash
   git clone <repository-url>
   cd Physical_AI
   ```

2. **Navigate to backend directory**
   ```bash
   cd backend
   ```

3. **Install dependencies with UV**
   ```bash
   uv venv
   source .venv/bin/activate  # On Windows: .venv\Scripts\activate
   uv pip install cohere qdrant-client beautifulsoup4 requests python-dotenv
   ```

4. **Set environment variables**
   Create a `.env` file in the backend directory:
   ```env
   COHERE_API_KEY=your_cohere_api_key_here
   QDRANT_URL=your_qdrant_url_here  # Optional, defaults to localhost
   QDRANT_API_KEY=your_qdrant_api_key_here  # Optional
   TARGET_URL=https://physical-ai-ivory.vercel.app/
   ```

## Usage

1. **Run the embedding pipeline**
   ```bash
   python main.py
   ```

2. **Monitor the process**
   The script will:
   - Discover all URLs from the target site
   - Extract clean text from each page
   - Chunk the text appropriately
   - Generate embeddings using Cohere
   - Store embeddings in Qdrant with metadata

## Verification

1. **Check Qdrant collection**
   Verify that a collection named "rag_embedding" has been created
   Check that vectors have been stored with proper metadata

2. **Test search functionality**
   You can perform similarity searches against the stored embeddings

## Troubleshooting

- **API Rate Limits**: If you encounter rate limit errors, reduce the concurrency or add delays
- **Network Issues**: Ensure the target URL is accessible and your network connection is stable
- **Memory Issues**: For large sites, consider processing in smaller batches