# Data Model: Retrieval from Qdrant

## Query Request Entity

**Attributes**:
- query_text: str (required) - The search query from the user
- top_k: int (default: 5) - Number of results to return
- min_score: float (optional) - Minimum similarity score threshold
- filters: dict (optional) - Additional filters for refined search

**Validation Rules**:
- query_text must not be empty
- top_k must be between 1 and 100
- min_score must be between 0.0 and 1.0 if provided

## Retrieval Result Entity

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
- chunk_id must be unique within the result set

## Formatted Output Entity

**Attributes**:
- query: str (required) - Original query text
- results: list[RetrievalResult] (required) - List of matching results
- total_results: int (required) - Total number of results found
- processing_time: float (required) - Time taken to process the query in seconds
- timestamp: float (required) - Unix timestamp when query was processed

**Validation Rules**:
- query must not be empty
- results must be a valid list
- total_results must match the length of results list
- processing_time must be non-negative

## Qdrant Search Parameters

**Attributes**:
- collection_name: str (default: "rag_embedding") - Name of the Qdrant collection to search
- vector_name: str (optional) - Name of the vector field to search
- with_payload: bool (default: True) - Whether to return payload data
- with_vectors: bool (default: False) - Whether to return vector data

**Validation Rules**:
- collection_name must exist in Qdrant
- vector_name must be valid if specified