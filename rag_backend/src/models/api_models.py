from typing import Optional
from pydantic import BaseModel, Field

class QueryRequest(BaseModel):
    query: str = Field(..., description="The user's question.")
    session_id: Optional[str] = Field(None, description="A session identifier for conversation history.")

class QueryResponse(BaseModel):
    answer: str = Field(..., description="The generated answer from the RAG model.")
    source: str = Field(..., description="The chapter and section the answer was drawn from, for citation.")
