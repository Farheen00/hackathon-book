# Data Model: RAG Agent with OpenAI Agents SDK + Gemini

## Agent Query Entity

**Attributes**:
- query_text: str (required) - The search query from the user
- top_k: int (default: 5) - Number of context chunks to retrieve
- min_score: float (optional) - Minimum similarity score for retrieved chunks

**Validation Rules**:
- query_text must not be empty
- top_k must be between 1 and 10
- min_score must be between 0.0 and 1.0 if provided

## Retrieved Context Entity

**Attributes**:
- content: str (required) - The retrieved text content
- similarity_score: float (required) - Relevance score from similarity search (0.0 to 1.0)
- source_url: str (required) - URL where the content originated
- chunk_id: str (required) - Identifier for the specific text chunk
- metadata: dict (optional) - Additional contextual information

**Validation Rules**:
- content must not be empty
- similarity_score must be between 0.0 and 1.0
- source_url must be a valid URL format

## Agent Response Entity

**Attributes**:
- answer: str (required) - The generated answer from the agent
- sources: list[str] (required) - List of source URLs used in answer generation
- matched_chunks: list[dict] (required) - Chunks that were used to generate the answer
- confidence: float (optional) - Confidence score for the answer (0.0 to 1.0)
- processing_time: float (required) - Time taken to process the query in seconds
- timestamp: float (required) - Unix timestamp when query was processed

**Validation Rules**:
- answer must not be empty
- sources must be a valid list of URLs
- matched_chunks must be a valid list of context entities
- confidence must be between 0.0 and 1.0 if provided

## Agent Configuration Entity

**Attributes**:
- client_type: str (default: "gemini") - Type of external client being used
- model_name: str (default: "gemini-pro") - Name of the model to use
- temperature: float (default: 0.7) - Creativity parameter for generation (0.0 to 1.0)
- max_tokens: int (default: 1024) - Maximum tokens for the response

**Validation Rules**:
- client_type must be a supported external client
- temperature must be between 0.0 and 1.0
- max_tokens must be positive