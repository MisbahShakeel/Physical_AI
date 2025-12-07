# Vercel Deployment Guide for Physical AI and Robotics Learning Framework

## Backend API Deployment

The backend API (FastAPI application) needs to be deployed separately from the frontend. Here are the options:

### Option 1: Deploy Backend to Vercel Functions (Recommended for simple mock API)

1. Create a new directory `api` in your frontend project root
2. Create API routes that mirror your backend functionality

### Option 2: Deploy Backend to a Different Service
- Deploy the FastAPI backend to services like:
  - Railway
  - Render
  - Heroku
  - AWS
  - Google Cloud

## Frontend Configuration for Vercel

When deploying the Docusaurus frontend to Vercel:

1. Update the environment variables in Vercel dashboard:
   - `REACT_APP_API_BASE_URL`: Set this to your deployed backend URL
   - Example: If your backend is deployed at `https://my-backend.onrender.com`, set this value

2. The chat widget is already configured to handle different environments:
   - In development: Uses `http://localhost:8000`
   - In production: Uses the value of `REACT_APP_API_BASE_URL` environment variable

## Deployment Steps

### For Frontend (Docusaurus) on Vercel:
1. Push your code to a GitHub repository
2. Connect the repository to Vercel
3. In Vercel dashboard, set environment variable:
   - Key: `REACT_APP_API_BASE_URL`
   - Value: Your backend API URL (e.g., `https://your-backend-app.onrender.com`)

### For Backend API:
Choose one of these approaches:

#### Approach A: Deploy to Render
1. Create a `Dockerfile` in your Backend directory:
```Dockerfile
FROM python:3.9-slim

WORKDIR /app

COPY requirements.txt .
RUN pip install -r requirements.txt

COPY . .

CMD ["uvicorn", "fastapi_endpoints:app", "--host", "0.0.0.0", "--port", "10000"]
```

2. Create a `render.yaml`:
```yaml
services:
  - type: web
    name: physical-ai-backend
    env: python
    buildCommand: pip install -r requirements.txt
    startCommand: uvicorn fastapi_endpoints:app --host 0.0.0.0 --port $PORT
    envVars:
      - key: OPENAI_API_KEY
        value: your_openai_api_key
      - key: QDRANT_URL
        value: your_qdrant_url
      - key: QDRANT_API_KEY
        value: your_qdrant_api_key
      - key: POSTGRES_DSN
        value: your_postgres_connection_string
```

#### Approach B: Deploy to Railway
1. Create a `Procfile` in your Backend directory:
```
web: uvicorn fastapi_endpoints:app --host 0.0.0.0 --port $PORT
```

2. Add environment variables in Railway dashboard

## Environment-Specific Configuration

The chat widget in `src/components/ChatWidget/index.tsx` is already configured to handle different environments:

```typescript
const apiBaseUrl = typeof window !== 'undefined'
  ? (process.env.REACT_APP_API_BASE_URL ||
     (window.location.hostname === 'localhost' ? 'http://localhost:8000' : ''))
  : 'http://localhost:8000';
```

## Troubleshooting

1. **CORS Issues**: Make sure your backend allows requests from your frontend domain
2. **Environment Variables**: Verify that `REACT_APP_API_BASE_URL` is set correctly in Vercel
3. **API Endpoints**: Ensure your deployed backend has the same endpoints as your local version

## Alternative: Frontend-Only Deployment with Mock API

If you want to deploy a frontend-only version for demonstration purposes:

1. Update the chat widget to use the mock API logic directly in the frontend
2. This would provide sample responses without requiring a backend

This approach is suitable for showcasing the UI/UX without backend functionality.