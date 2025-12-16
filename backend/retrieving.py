import os
import sys
import time
import argparse
from typing import List, Dict, Any, Optional
import json
import requests
from bs4 import BeautifulSoup
import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models
from dotenv import load_dotenv
import logging

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Initialize clients
co = cohere.Client(os.getenv("COHERE_API_KEY"))

# Check if using cloud Qdrant instance
qdrant_url = os.getenv("QDRANT_URL")
qdrant_api_key = os.getenv("QDRANT_API_KEY")

if qdrant_url and qdrant_api_key:
    # Use cloud instance
    qdrant_client = QdrantClient(
        url=qdrant_url,
        api_key=qdrant_api_key
    )
else:
    # Use local instance
    qdrant_host = os.getenv("QDRANT_HOST", "localhost")
    qdrant_port = int(os.getenv("QDRANT_PORT", 6333))
    qdrant_client = QdrantClient(host=qdrant_host, port=qdrant_port)

def embed_query(query_text: str) -> List[float]:
    """
    Generate embedding vector for the query text using Cohere.
    """
    try:
        response = co.embed(
            texts=[query_text],
            model='embed-multilingual-v2.0'  # Using same model as ingestion
        )
        return response.embeddings[0]
    except Exception as e:
        logger.error(f"Error generating embedding for query: {str(e)}")
        # Return a zero vector as fallback
        return [0.0] * 768  # Assuming 768-dim embedding as default

def search_qdrant(query_embedding: List[float], top_k: int = 5, min_score: Optional[float] = None) -> List[Dict]:
    """
    Search Qdrant collection for similar vectors and return results with metadata.
    """
    try:
        # Prepare search filter if minimum score is specified
        search_filter = None
        if min_score is not None:
            search_filter = models.Filter(
                must=[
                    models.FieldCondition(
                        key="similarity_score",
                        range=models.Range(gte=min_score)
                    )
                ]
            )

        # Perform the search
        search_results = qdrant_client.search(
            collection_name="rag_embedding",
            query_vector=query_embedding,
            limit=top_k,
            with_payload=True,
            with_vectors=False,
            score_threshold=min_score  # Additional threshold on the similarity score
        )

        # Format results
        formatted_results = []
        for result in search_results:
            formatted_result = {
                "content": result.payload.get("content", ""),
                "similarity_score": result.score,
                "source_url": result.payload.get("source_url", ""),
                "chunk_id": result.id,  # Qdrant point ID
                "metadata": {k: v for k, v in result.payload.items() if k not in ["content", "source_url"]}
            }
            formatted_results.append(formatted_result)

        logger.info(f"Found {len(formatted_results)} results from Qdrant")
        return formatted_results

    except Exception as e:
        logger.error(f"Error searching Qdrant: {str(e)}")
        return []

def validate_retrieval_results(results: List[Dict]) -> List[Dict]:
    """
    Validate and format retrieval results ensuring content and metadata integrity.
    """
    validated_results = []

    for result in results:
        # Validate required fields
        if not result.get("content") or not result.get("source_url"):
            logger.warning("Skipping result with missing required fields")
            continue

        # Validate similarity score is within expected range
        score = result.get("similarity_score", 0.0)
        if not (0.0 <= score <= 1.0):
            logger.warning(f"Similarity score out of range: {score}")
            # Adjust score to be within range
            result["similarity_score"] = max(0.0, min(1.0, score))

        validated_results.append(result)

    logger.info(f"Validated {len(validated_results)} results")
    return validated_results

def retrieve_and_print(query: str, top_k: int = 5, min_score: Optional[float] = None) -> Dict:
    """
    Complete retrieval pipeline: query → embedding → search → validation → formatted output.
    """
    start_time = time.time()

    logger.info(f"Starting retrieval for query: '{query}'")

    # Step 1: Generate embedding for the query
    query_embedding = embed_query(query)
    if not query_embedding or len(query_embedding) == 0:
        logger.error("Failed to generate query embedding")
        return {
            "query": query,
            "results": [],
            "total_results": 0,
            "processing_time": time.time() - start_time,
            "timestamp": time.time(),
            "error": "Failed to generate query embedding"
        }

    # Step 2: Search Qdrant for similar vectors
    raw_results = search_qdrant(query_embedding, top_k, min_score)
    if not raw_results:
        logger.warning("No results found from Qdrant")

    # Step 3: Validate and format results
    validated_results = validate_retrieval_results(raw_results)

    # Step 4: Prepare final output
    output = {
        "query": query,
        "results": validated_results,
        "total_results": len(validated_results),
        "processing_time": time.time() - start_time,
        "timestamp": time.time()
    }

    logger.info(f"Retrieval completed in {output['processing_time']:.2f}s")

    # Print results in a readable format
    print(f"\nQuery: {query}")
    print(f"Processing time: {output['processing_time']:.2f}s")
    print(f"Total results found: {output['total_results']}")
    print("-" * 80)

    for i, result in enumerate(validated_results, 1):
        print(f"\nResult {i}: (Score: {result['similarity_score']:.3f})")
        print(f"Source: {result['source_url']}")
        print(f"Content preview: {result['content'][:200]}{'...' if len(result['content']) > 200 else ''}")
        print("-" * 40)

    return output

def main():
    """
    Main function to handle command-line arguments and execute retrieval.
    """
    parser = argparse.ArgumentParser(description="Retrieve information from Qdrant based on a query")
    parser.add_argument("query", nargs='?', help="The search query")
    parser.add_argument("--top-k", type=int, default=5, help="Number of results to return (default: 5)")
    parser.add_argument("--min-score", type=float, help="Minimum similarity score threshold")

    args = parser.parse_args()

    # If no query provided via command line, prompt user
    if not args.query:
        query = input("Enter your search query: ")
    else:
        query = args.query

    # Execute retrieval
    results = retrieve_and_print(query, args.top_k, args.min_score)

    # Print full JSON output at the end
    print("\nFull JSON output:")
    print(json.dumps(results, indent=2, default=str))

if __name__ == "__main__":
    main()