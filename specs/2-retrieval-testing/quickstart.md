# Quickstart Guide: Retrieval from Qdrant

## Prerequisites

- Python 3.8+
- UV package manager
- Cohere API key
- Qdrant instance (local or remote) with embeddings already stored
- Environment variables configured in `.env` file

## Setup

1. Ensure you have the required environment variables in your `.env` file:
   ```
   COHERE_API_KEY=your_cohere_api_key_here
   QDRANT_URL=your_qdrant_cloud_url_here (or use QDRANT_HOST/QDRANT_PORT for local)
   QDRANT_API_KEY=your_qdrant_api_key_here (if using cloud instance)
   ```

2. Navigate to the backend directory:
   ```bash
   cd backend
   ```

## Running the Retrieval

1. Execute the retrieval script directly:
   ```bash
   python retrieving.py "your search query here"
   ```

2. Or import and use the retrieval functions in your code:
   ```python
   from retrieving import retrieve_and_print

   results = retrieve_and_print("your search query", top_k=5)
   print(results)
   ```

## Configuration

The retrieval can be configured via:
- Function parameters (top_k, min_score)
- Environment variables in the `.env` file

## Example Usage

```bash
# Basic retrieval
python retrieving.py "What is ROS2?"

# Retrieval with custom result count
python retrieving.py "machine learning" --top-k 10

# Using the module in code
from retrieving import retrieve_and_print

results = retrieve_and_print("Explain digital twin concepts", top_k=3)
print(f"Found {results['total_results']} results in {results['processing_time']:.2f}s")
for result in results['results']:
    print(f"Score: {result['similarity_score']:.3f}")
    print(f"Content: {result['content'][:100]}...")
    print(f"Source: {result['source_url']}")
    print("---")
```

## Troubleshooting

- If you get "No results found" errors, verify that embeddings have been stored in Qdrant
- If you get API rate limit errors, add delays between requests
- If similarity scores are low, consider adjusting the min_score threshold
- If Qdrant connection fails, verify your connection settings