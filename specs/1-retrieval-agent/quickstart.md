# Quickstart Guide: Retrieval-Enabled Agent

**Feature**: 1-retrieval-agent
**Date**: 2025-12-17

## Overview
This guide provides a quick introduction to setting up and running the retrieval-enabled OpenAI agent with Qdrant integration.

## Prerequisites
- Python 3.8+
- OpenAI API key
- Qdrant instance with embedded book content
- Access to necessary environment variables

## Setup

### 1. Environment Variables
Create a `.env` file in the backend directory with the following variables:

```bash
QDRANT_URL=https://your-qdrant-instance.com
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_COLLECTION_NAME=your_collection_name
OPENAI_API_KEY=your_openai_api_key_here
```

### 2. Install Dependencies
```bash
pip install openai qdrant-client python-dotenv pydantic
```

### 3. File Structure
Ensure the following files are in place:
```
backend/
├── agent.py              # Main agent implementation
├── config.py             # Configuration management
├── retrieval_tool.py     # Custom Qdrant retrieval tool
└── .env                  # Environment variables (not committed)
```

## Running the Agent

### 1. Basic Usage
```python
from agent import OpenAIAgent

# Initialize the agent
agent = OpenAIAgent()

# Query the agent
response = agent.query("What are the key principles of humanoid robotics?")
print(response)
```

### 2. Advanced Options
```python
# Query with specific options
response = agent.query(
    query="Explain the fundamentals of movement in humanoid robots",
    top_k=7,  # Retrieve 7 chunks instead of default 5
    similarity_threshold=0.6  # Adjust similarity threshold
)
```

## API Usage
Once deployed, interact with the agent via HTTP:

```bash
curl -X POST http://localhost:8000/api/agent/query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What are the key principles of humanoid robotics?",
    "options": {
      "top_k": 5,
      "similarity_threshold": 0.5
    }
  }'
```

## Troubleshooting

### Common Issues
1. **Qdrant Connection Failed**: Verify `QDRANT_URL` and `QDRANT_API_KEY` in your environment
2. **OpenAI API Error**: Check that `OPENAI_API_KEY` is properly set
3. **No Results Found**: Adjust `similarity_threshold` to a lower value or check Qdrant collection

### Verifying Setup
Run the health check endpoint:
```bash
curl http://localhost:8000/api/agent/health
```

## Next Steps
- Review the detailed implementation plan in `specs/1-retrieval-agent/plan.md`
- Check the data model in `specs/1-retrieval-agent/data-model.md`
- Examine the API contracts in `specs/1-retrieval-agent/contracts/`