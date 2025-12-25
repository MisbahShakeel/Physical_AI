# Embedding Pipeline Setup

## Prerequisites

1. **Docker and Docker Compose** - Required to run Qdrant vector database
2. **Cohere API Key** - Required for generating embeddings

## Setup Instructions

### 1. Start Qdrant Database

```bash
docker-compose up -d
```

This will start Qdrant on `http://localhost:6333`

### 2. Environment Configuration

Copy the example environment file and add your API keys:

```bash
# The .env file is already configured, but you need to update it with your actual keys
```

Update the `.env` file with your actual API keys:
- `COHERE_API_KEY`: Your Cohere API key from https://dashboard.cohere.com/api-keys

### 3. Install Dependencies

```bash
cd backend
pip install -r requirements.txt
```

If no requirements.txt exists, install the required packages:

```bash
pip install requests beautifulsoup4 cohere qdrant-client python-dotenv
```

### 4. Run the Embedding Pipeline

```bash
cd backend
python main.py
```

## What the Pipeline Does

1. Crawls the target website (configured via `TARGET_URL` in .env)
2. Extracts clean text content from all discovered pages
3. Chunks the text into manageable pieces
4. Generates embeddings using Cohere
5. Stores the embeddings in Qdrant with metadata

## Troubleshooting

- If you get connection errors, ensure Qdrant is running on `http://localhost:6333`
- If embeddings fail, verify your Cohere API key is correct and has sufficient quota
- Check the logs for specific error messages

## Verification

After running the pipeline, you can verify embeddings were created by visiting:
- Qdrant UI: http://localhost:6333/dashboard
- Look for the "rag_embedding" collection with points