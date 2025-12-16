import os
import sys
import time
import argparse
from typing import List, Dict, Any, Optional
import json
import logging
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Import Google Generative AI - this will be used as the external client
try:
    import google.generativeai as genai
    from google.generativeai.types import GenerationConfig
except ImportError:
    logger.error("Please install google-generativeai: pip install google-generativeai")
    sys.exit(1)

# Import other required modules
try:
    import google.generativeai as genai
    from google.generativeai.types import GenerationConfig
except ImportError:
    logger.error("Please install google-generativeai: pip install google-generativeai")
    raise

# Import Cohere and Qdrant clients from existing retrieving module
sys.path.append(os.path.join(os.path.dirname(__file__)))
from retrieving import retrieve_and_print

# Configure the Gemini client
genai.configure(api_key=os.getenv("GEMINI_API_KEY"))
model = genai.GenerativeModel('gemini-pro')

def configure_gemini_client():
    """
    Configure and return the Gemini client using environment variables.
    """
    api_key = os.getenv("GEMINI_API_KEY")
    if not api_key:
        raise ValueError("GEMINI_API_KEY environment variable is required")

    genai.configure(api_key=api_key)
    return genai.GenerativeModel('gemini-pro')

def retrieve_context(query: str, top_k: int = 5) -> List[Dict]:
    """
    Retrieve relevant context from Qdrant based on the query.
    Reuse the retrieval logic from the existing retrieving module.
    """
    # Use the existing retrieve_and_print function but only get the results
    results = retrieve_and_print(query, top_k=top_k)
    return results.get("results", [])

def build_agent_prompt(query: str, context: List[Dict]) -> str:
    """
    Build a prompt combining the user query with retrieved context.
    """
    if not context:
        return f"Question: {query}\n\nI couldn't find any relevant information in the knowledge base to answer this question. Please answer based on your general knowledge or say that you don't have enough information."

    # Format the context into a readable format for the LLM
    context_text = "\n\n".join([
        f"Source: {chunk['source_url']}\nContent: {chunk['content'][:1000]}"  # Limit content length
        for chunk in context
    ])

    prompt = f"""
    You are an AI assistant with access to a knowledge base. Use the provided context to answer the user's question.
    If the context doesn't contain relevant information, say so.

    Context:
    {context_text}

    Question: {query}

    Please provide a comprehensive answer based on the context, and list the sources you used.
    """

    return prompt.strip()

def run_agent(query: str, top_k: int = 5, temperature: float = 0.7) -> Dict:
    """
    Execute the complete agent flow: retrieve → build prompt → generate response.
    """
    start_time = time.time()

    logger.info(f"Starting agent for query: '{query}'")

    try:
        # Step 1: Retrieve context from Qdrant
        context_chunks = retrieve_context(query, top_k)
        logger.info(f"Retrieved {len(context_chunks)} context chunks")

        # Step 2: Build the agent prompt with context
        prompt = build_agent_prompt(query, context_chunks)

        # Step 3: Configure the Gemini client
        gemini_model = configure_gemini_client()

        # Step 4: Generate response using Gemini
        generation_config = GenerationConfig(
            temperature=temperature,
            max_output_tokens=1024,
        )

        response = gemini_model.generate_content(
            prompt,
            generation_config=generation_config
        )

        # Extract the answer from the response
        answer = response.text if response.text else "I couldn't generate a response for your query."

        # Extract sources from the context chunks
        sources = list(set([chunk["source_url"] for chunk in context_chunks if chunk["source_url"]]))

        # Prepare the final response
        result = {
            "answer": answer,
            "sources": sources,
            "matched_chunks": context_chunks,
            "processing_time": time.time() - start_time,
            "timestamp": time.time()
        }

        logger.info(f"Agent completed in {result['processing_time']:.2f}s")
        return result

    except Exception as e:
        logger.error(f"Error in agent execution: {str(e)}")
        return {
            "answer": "An error occurred while processing your request.",
            "sources": [],
            "matched_chunks": [],
            "processing_time": time.time() - start_time,
            "timestamp": time.time(),
            "error": str(e)
        }

def main():
    """
    Main function to handle command-line arguments and execute the agent.
    """
    parser = argparse.ArgumentParser(description="RAG Agent with Gemini")
    parser.add_argument("query", nargs='?', help="The query for the agent")
    parser.add_argument("--top-k", type=int, default=5, help="Number of context chunks to retrieve (default: 5)")
    parser.add_argument("--temperature", type=float, default=0.7, help="Temperature for generation (default: 0.7)")

    args = parser.parse_args()

    # If no query provided via command line, prompt user
    if not args.query:
        query = input("Enter your query for the agent: ")
    else:
        query = args.query

    # Execute the agent
    result = run_agent(query, top_k=args.top_k, temperature=args.temperature)

    # Print the result
    print("\n" + "="*80)
    print("RAG AGENT RESPONSE")
    print("="*80)
    print(f"Query: {query}")
    print(f"Processing time: {result['processing_time']:.2f}s")
    print(f"Answer: {result['answer']}")

    if result.get('sources'):
        print(f"\nSources: {', '.join(result['sources'])}")

    print(f"\nMatched chunks: {len(result['matched_chunks'])}")
    for i, chunk in enumerate(result['matched_chunks'], 1):
        print(f"  {i}. {chunk['source_url']} (Score: {chunk['similarity_score']:.3f})")
        print(f"     Preview: {chunk['content'][:100]}...")

    # Print full JSON output at the end
    print("\nFull JSON response:")
    print(json.dumps(result, indent=2, default=str))

if __name__ == "__main__":
    main()