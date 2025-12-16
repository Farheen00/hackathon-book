# Quickstart Guide: Backend RAG Pipeline

## Prerequisites

- Python 3.8+
- UV package manager
- Cohere API key
- Qdrant instance (local or remote)

## Setup

1. Navigate to the backend directory:
   ```bash
   cd backend
   ```

2. Install dependencies using UV:
   ```bash
   uv init
   uv add qdrant-client cohere requests beautifulsoup4 python-dotenv
   ```

3. Create a `.env` file with your API keys:
   ```
   COHERE_API_KEY=your_cohere_api_key_here
   QDRANT_HOST=your_qdrant_host_here (optional, defaults to localhost)
   QDRANT_PORT=6333 (optional, defaults to 6333)
   ```

## Running the Pipeline

1. Execute the main script:
   ```bash
   python main.py
   ```

2. The pipeline will:
   - Extract all URLs from the target website
   - Process each URL to extract clean text
   - Chunk the text into manageable pieces
   - Generate embeddings using Cohere
   - Store embeddings in Qdrant with metadata

## Configuration

The pipeline can be configured via environment variables in the `.env` file:
- `COHERE_API_KEY`: Your Cohere API key for embedding generation
- `QDRANT_HOST`: Host address of your Qdrant instance (default: localhost)
- `QDRANT_PORT`: Port of your Qdrant instance (default: 6333)
- `TARGET_URL`: The website URL to process (default: https://hackathon-book-roan.vercel.app/)

## Troubleshooting

- If you get API rate limit errors, add delays between Cohere API calls
- If Qdrant connection fails, verify host and port settings
- If text extraction fails for certain pages, check the URL format