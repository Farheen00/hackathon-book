# Data Model: Production RAG Agent with FastAPI + Docusaurus Chat UI

## Chat Message Entity

**Attributes**:
- id: str (required) - Unique identifier for the message
- content: str (required) - The message content
- role: str (required) - Sender role ("user" or "assistant")
- timestamp: float (required) - Unix timestamp when message was created

**Validation Rules**:
- id must be unique within the conversation
- content must not be empty
- role must be either "user" or "assistant"
- timestamp must be a valid Unix timestamp

## API Request Entity

**Attributes**:
- query: str (required) - The user's question/query
- top_k: int (optional, default: 5) - Number of context chunks to retrieve
- temperature: float (optional, default: 0.7) - Generation temperature parameter

**Validation Rules**:
- query must not be empty
- top_k must be between 1 and 10 if provided
- temperature must be between 0.0 and 1.0 if provided

## API Response Entity

**Attributes**:
- answer: str (required) - The generated answer from the agent
- sources: list[str] (required) - List of source URLs used in answer generation
- matched_chunks: list[dict] (required) - Chunks that were used to generate the answer
- processing_time: float (required) - Time taken to process the query in seconds
- timestamp: float (required) - Unix timestamp when response was generated

**Validation Rules**:
- answer must not be empty
- sources must be a valid list of URLs
- matched_chunks must be a valid list of chunk objects
- processing_time must be non-negative

## Chat Session Entity

**Attributes**:
- session_id: str (required) - Unique identifier for the chat session
- messages: list[ChatMessage] (required) - List of messages in the session
- created_at: float (required) - Unix timestamp when session was created
- last_accessed: float (required) - Unix timestamp of last session access

**Validation Rules**:
- session_id must be unique
- messages must be a valid list of ChatMessage entities
- created_at must be a valid timestamp
- last_accessed must be a valid timestamp