# Data Model: Backend RAG Pipeline

## Document Chunk Entity

**Attributes**:
- content: str (required) - The text content to be embedded
- source_url: str (required) - The URL where the content was extracted from
- chunk_index: int (required) - The position of this chunk in the original document
- metadata: dict (optional) - Additional contextual information

**Validation Rules**:
- content must not be empty
- source_url must be a valid URL format
- chunk_index must be non-negative

## Qdrant Point Entity

**Attributes**:
- id: str (required) - Unique identifier for the vector point
- vector: list[float] (required) - The embedding vector from Cohere
- payload: dict (required) - Contains content, source_url, and metadata

**Validation Rules**:
- id must be unique within the collection
- vector must have consistent dimensions
- payload must contain required metadata fields

## URL Entity

**Attributes**:
- url: str (required) - The URL to process
- processed: bool (default: False) - Whether the URL has been processed
- error: str (optional) - Error message if processing failed

**Validation Rules**:
- url must be a valid URL format
- url must be within the target domain

## Collection Configuration

**Attributes**:
- name: str (default: "rag_embedding") - Name of the Qdrant collection
- vector_size: int (required) - Dimension of the embedding vectors
- distance: str (default: "Cosine") - Distance metric for similarity search

**Validation Rules**:
- name must follow Qdrant naming conventions
- vector_size must match the embedding model output