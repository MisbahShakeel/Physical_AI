# Physical AI and Robotics Learning Framework - Backend API

## Overview
This backend API provides a RAG (Retrieval-Augmented Generation) system that connects to Qdrant vector database and uses Google Gemini for intelligent responses based on textbook content.

## Prerequisites
- Python 3.8+
- Google Gemini API key
- Cohere API key
- Qdrant vector database access

## Setup

### 1. Environment Variables
Create a `.env` file in the backend directory with the following variables:

```env
GEMINI_API_KEY=your_google_gemini_api_key
COHERE_API_KEY=your_cohere_api_key
QDRANT_URL=your_qdrant_url  # e.g., https://your-cluster-url.gcp.cloud.qdrant.io:6333
QDRANT_API_KEY=your_qdrant_api_key  # optional if using local Qdrant
QDRANT_COLLECTION_NAME=rag_embedding
```

### 2. Install Dependencies
```bash
pip install -r requirements.txt
```

## Running the API

### Method 1: Direct Python
```bash
cd backend
python api.py
```

### Method 2: Using the start script (Windows)
```bash
cd backend
start_api.bat
```

The API will start on `http://localhost:8000`

## API Endpoints

### POST /query
Submit a query to search the textbook content
- Request body: `{"query": "your question", "top_k": 5, "collection_name": "rag_embedding"}`
- Response: Contains the answer, citations, and metadata

### GET /health
Check the health status of the service

### GET /config
Get API configuration and capabilities

## Frontend Integration
The frontend (Docusaurus site) automatically connects to the backend API at `/query` endpoint when available. If the backend is not available, it falls back to mock responses.

## Troubleshooting
- Make sure all required API keys are set in the `.env` file
- Verify Qdrant connection parameters
- Check that the Qdrant collection contains the embedded content
- The API uses CORS headers to allow requests from frontend applications