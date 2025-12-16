# Quickstart Guide: RAG Agent with OpenAI Agents SDK + Gemini

## Prerequisites

- Python 3.8+
- UV package manager
- Gemini API key
- Qdrant instance with embeddings already stored
- Cohere API key
- Environment variables configured in `.env` file

## Setup

1. Ensure you have the required environment variables in your `.env` file:
   ```
   GEMINI_API_KEY=your_gemini_api_key_here
   COHERE_API_KEY=your_cohere_api_key_here
   QDRANT_URL=your_qdrant_cloud_url_here (or use QDRANT_HOST/QDRANT_PORT for local)
   QDRANT_API_KEY=your_qdrant_api_key_here (if using cloud instance)
   ```

2. Install the required dependencies:
   ```bash
   cd backend
   uv add openai google-generativeai
   ```

3. Navigate to the backend directory:
   ```bash
   cd backend
   ```

## Running the Agent

1. Execute the agent script directly:
   ```bash
   python agent.py "your question here"
   ```

2. Or import and use the agent functions in your code:
   ```python
   from agent import run_agent

   result = run_agent("your question")
   print(result)
   ```

## Configuration

The agent can be configured via:
- Function parameters (top_k, temperature)
- Environment variables in the `.env` file

## Example Usage

```bash
# Basic agent query
python agent.py "What is the main concept of digital twins?"

# Using the module in code
from agent import run_agent

result = run_agent("Explain ROS2 concepts from the documentation")
print(f"Answer: {result['answer']}")
print(f"Sources: {result['sources']}")
print(f"Processing time: {result['processing_time']:.2f}s")
```

## Troubleshooting

- If you get "API key not found" errors, verify your GEMINI_API_KEY in the .env file
- If context retrieval fails, verify that embeddings have been stored in Qdrant
- If you get API rate limit errors, consider adding delays between requests
- If Qdrant connection fails, verify your connection settings