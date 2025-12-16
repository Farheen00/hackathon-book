import os
import time
import logging
from typing import Optional, List, Dict, Any
from fastapi import FastAPI, HTTPException, BackgroundTasks
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field
from contextlib import asynccontextmanager
import asyncio

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Import the agent functionality
import sys
sys.path.append(os.path.join(os.path.dirname(__file__)))
from agent import run_agent

# Define request and response models
class AskRequest(BaseModel):
    query: str = Field(..., min_length=1, max_length=2000, description="The user's question/query")
    top_k: int = Field(default=5, ge=1, le=10, description="Number of context chunks to retrieve")
    temperature: float = Field(default=0.7, ge=0.0, le=1.0, description="Generation temperature parameter")

class AskResponse(BaseModel):
    answer: str
    sources: List[str]
    matched_chunks: List[Dict[str, Any]]
    processing_time: float
    timestamp: float

class HealthResponse(BaseModel):
    status: str
    timestamp: float

# Track application lifecycle
@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup
    logger.info("Starting RAG Agent API server")
    yield
    # Shutdown
    logger.info("Shutting down RAG Agent API server")

# Create FastAPI app
app = FastAPI(
    title="RAG Agent API",
    description="API for RAG Agent with Qdrant and Gemini integration",
    version="1.0.0",
    lifespan=lifespan
)

# Add CORS middleware for Docusaurus frontend
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific domain
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
    # In production, be more specific about allowed origins
)

@app.get("/health", response_model=HealthResponse)
async def health_check():
    """
    Health check endpoint to verify the server is running.
    """
    return HealthResponse(
        status="healthy",
        timestamp=time.time()
    )

@app.post("/ask", response_model=AskResponse)
async def ask_agent(request: AskRequest):
    """
    Submit a query to the RAG agent and receive a response with answer, sources, and matched chunks.
    """
    try:
        start_time = time.time()

        # Run the agent with the provided parameters
        result = run_agent(
            query=request.query,
            top_k=request.top_k,
            temperature=request.temperature
        )

        # Calculate total processing time including the API overhead
        total_processing_time = time.time() - start_time

        # Check if there was an error in the agent execution
        if result.get("error"):
            logger.error(f"Agent error: {result['error']}")
            raise HTTPException(status_code=500, detail=result["error"])

        # Create and return the response
        response = AskResponse(
            answer=result["answer"],
            sources=result["sources"],
            matched_chunks=result["matched_chunks"],
            processing_time=total_processing_time,
            timestamp=result["timestamp"]
        )

        logger.info(f"Query processed successfully in {total_processing_time:.2f}s")
        return response

    except HTTPException:
        # Re-raise HTTP exceptions as-is
        raise
    except Exception as e:
        logger.error(f"Unexpected error in ask_agent: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Internal server error: {str(e)}")

@app.get("/")
async def root():
    """
    Root endpoint providing basic information about the API.
    """
    return {
        "message": "RAG Agent API",
        "version": "1.0.0",
        "endpoints": {
            "POST /ask": "Submit a query to the RAG agent",
            "GET /health": "Health check endpoint"
        }
    }

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "main:app",
        host="0.0.0.0",
        port=8000,
        reload=True,
        log_level="info"
    )