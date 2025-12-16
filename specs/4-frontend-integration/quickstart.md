# Quickstart Guide: Production RAG Agent with FastAPI + Docusaurus Chat UI

## Prerequisites

- Python 3.8+
- Node.js and npm for Docusaurus
- UV package manager
- Gemini API key
- Qdrant instance with embeddings stored
- Cohere API key
- Environment variables configured in `.env` file

## Backend Setup

1. Ensure you have the required environment variables in your `.env` file:
   ```
   GEMINI_API_KEY=your_gemini_api_key_here
   COHERE_API_KEY=your_cohere_api_key_here
   QDRANT_URL=your_qdrant_cloud_url_here (or use QDRANT_HOST/QDRANT_PORT for local)
   QDRANT_API_KEY=your_qdrant_api_key_here (if using cloud instance)
   ```

2. Install backend dependencies:
   ```bash
   cd backend
   uv add fastapi uvicorn python-multipart google-generativeai
   ```

3. Start the FastAPI server:
   ```bash
   cd backend
   uvicorn main:app --reload --port 8000
   ```

## Frontend Setup

1. Navigate to the Docusaurus project root:
   ```bash
   cd ..
   ```

2. Install Docusaurus dependencies if not already done:
   ```bash
   npm install
   ```

3. Start the Docusaurus development server:
   ```bash
   npm run start
   ```

## API Usage

The backend exposes the following endpoints:
- `POST /ask` - Submit a query to the RAG agent
- `GET /health` - Check server health status

Example API call:
```bash
curl -X POST http://localhost:8000/ask \
  -H "Content-Type: application/json" \
  -d '{"query": "Your question here", "top_k": 5}'
```

## Chat UI

Once both servers are running, the chat UI will appear as a floating button in the bottom-right corner of the Docusaurus site. Click it to open the chat interface and start asking questions.

## Production Deployment

For production deployment:
1. Use a production ASGI server like Gunicorn instead of Uvicorn with reload
2. Set environment variables for production
3. Configure proper CORS settings for your domain
4. Set up logging and monitoring