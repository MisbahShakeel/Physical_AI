# Physical AI and Robotics Learning Framework - Frontend-Backend Integration

## Overview

This document describes the integration between the frontend (Docusaurus-based documentation site) and the backend (FastAPI-based RAG system) for the Physical AI and Robotics Learning Framework.

## Architecture

### Backend Components

The backend consists of:
- FastAPI application with endpoints for querying, health checks, and document ingestion
- Mock API for frontend integration testing (without external dependencies)
- Configuration management via Pydantic settings
- Database connections (mocked in the test environment)

### Frontend Components

The frontend consists of:
- Docusaurus-based documentation site
- React-based chat widget component
- CSS modules for styling
- TypeScript for type safety

## API Endpoints

### Main Endpoints

- `GET /` - Root endpoint confirming API is running
- `GET /health` - Health check endpoint
- `POST /query` - Query endpoint for textbook knowledge base
- `POST /ingest` - Document ingestion endpoint
- `POST /embed` - Embedding generation endpoint
- `POST /feedback` - Feedback submission endpoint

### Query Endpoint Request Format

```json
{
  "query": "your question here",
  "session_id": "unique session id",
  "selected_text": "optional selected text",
  "top_k": 10,
  "temperature": 0.1
}
```

### Query Endpoint Response Format

```json
{
  "response": "response text",
  "citations": [
    {
      "chapter_title": "chapter title",
      "section_title": "section title",
      "page_number": 123,
      "module": "module name"
    }
  ],
  "search_mode": "global or selected_text",
  "latency_ms": 123,
  "query_id": "unique query id"
}
```

## Frontend Integration

### Chat Widget Component

The chat widget is implemented as a React component with the following features:
- Message history display
- User input handling
- API communication with backend
- Loading indicators
- Citation formatting
- Error handling

### API Communication

The frontend communicates with the backend through:
- REST API calls using fetch
- Proper error handling and loading states
- Session management
- Response formatting

## Mock API for Testing

A mock API is provided for frontend integration testing without requiring external services:
- Simulated responses based on query content
- Mock citations generation
- Health check responses
- No dependency on external databases or services

## Environment Setup

### Backend

1. Install Python dependencies:
   ```bash
   pip install fastapi uvicorn openai qdrant-client asyncpg pydantic-settings
   ```

2. Set environment variables in `.env` file:
   ```env
   OPENAI_API_KEY=your_api_key
   QDRANT_URL=http://localhost:6333
   QDRANT_API_KEY=your_qdrant_key
   POSTGRES_DSN=postgresql://user:password@localhost/dbname
   ```

3. Start the backend server:
   ```bash
   cd Backend && python -m uvicorn fastapi_endpoints:app --host 0.0.0.0 --port 8000
   ```

### Frontend

1. Install dependencies:
   ```bash
   cd Frontend/Physical_AI && npm install
   ```

2. Start the development server:
   ```bash
   cd Frontend/Physical_AI && npm start
   ```

## Testing the Integration

1. Start the backend API server (either main API or mock API)
2. Verify the backend is running by accessing `http://localhost:8000/health`
3. Start the frontend development server
4. Use the chat widget to test API communication
5. Verify that responses are properly formatted and displayed

## Troubleshooting

### Common Issues

1. **CORS errors**: Ensure CORS middleware is properly configured in the backend
2. **Connection refused**: Verify the backend server is running and accessible
3. **API key errors**: Check that all required environment variables are set
4. **Database connection errors**: Ensure database services are running

### Debugging Steps

1. Check backend server logs for errors
2. Verify environment variables are properly set
3. Test API endpoints directly using tools like curl or Postman
4. Check frontend browser console for JavaScript errors
5. Verify network requests in browser developer tools