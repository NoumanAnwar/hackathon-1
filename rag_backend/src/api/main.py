import os
from fastapi import FastAPI, HTTPException
from rag_backend.src.models.api_models import QueryRequest, QueryResponse
from rag_backend.src.services.embedding_service import EmbeddingService
from rag_backend.src.services.vector_store import VectorStore
import google.generativeai as genai

# Configure Gemini API
try:
    api_key = os.getenv("GEMINI_API_KEY")
    if not api_key:
        raise ValueError("GEMINI_API_KEY environment variable not set.")
    genai.configure(api_key=api_key)
except ValueError as e:
    print(f"Error configuring Gemini API: {e}")
    # Depending on deployment, you might want to exit or handle gracefully
    # For now, we'll let the endpoint fail if a query is made without a key.

# Initialize services
embedding_service = EmbeddingService()
vector_store = VectorStore() # Uses default collection_name and persist_directory

app = FastAPI(
    title="Book RAG Backend API",
    version="1.0.0",
    description="API for querying the AI/Spec book content using a RAG model."
)

@app.get("/")
async def read_root():
    return {"message": "RAG Backend API is running"}

@app.post("/api/query", response_model=QueryResponse)
async def query_rag(request: QueryRequest):
    """
    Accepts a user query and returns a RAG-generated response.
    """
    if not os.getenv("GEMINI_API_KEY"):
        raise HTTPException(status_code=500, detail="GEMINI_API_KEY is not set. Cannot perform RAG query.")

    try:
        # 1. Generate embedding for the user query
        query_embedding = await embedding_service.get_embedding(request.query)

        # 2. Retrieve relevant documents from the vector store
        retrieved_docs = vector_store.query_documents(query_embedding, n_results=5) # Get top 5 results

        if not retrieved_docs:
            return QueryResponse(answer="Could not find relevant information in the book.", source="None")

        # 3. Construct prompt for LLM
        context = "\n".join([doc["content"] for doc in retrieved_docs])
        prompt_parts = [
            "You are a helpful assistant for a book about ROS 2 robotics. Answer the user's question ONLY based on the provided context from the book. If the answer is not in the context, state that you cannot answer from the book. Provide citations by mentioning the source_path from the context (e.g., 'Source: chapter1.md').\n\n",
            "Context from the book:\n",
            context,
            f"\n\nUser Question: {request.query}\n",
            "Answer:"
        ]
        
        # 4. Generate answer using Gemini LLM
        model = genai.GenerativeModel('gemini-pro') # Using gemini-pro for text generation
        response = await model.generate_content(prompt_parts)
        
        # Extract answer and source from retrieved docs
        generated_answer = response.text
        # A more sophisticated approach would parse the generated_answer for citations.
        # For this basic implementation, we'll just return the source of the top retrieved doc.
        main_source = retrieved_docs[0]["metadata"]["source_path"]

        return QueryResponse(answer=generated_answer, source=f"Source: {main_source}")

    except Exception as e:
        print(f"Error during RAG query: {e}")
        raise HTTPException(status_code=500, detail="An error occurred while processing your query.")
