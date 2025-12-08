# Phase 0 Research

**Objective**: Resolve open questions from the initial implementation plan to finalize the technical approach before detailed design.

## 1. RAG Backend Technology Stack

### Research Task
Define the specific technology stack for the Retrieval-Augmented Generation (RAG) backend. This includes selecting:
1.  A web framework for the API.
2.  A vector database for storing and retrieving text embeddings.
3.  A specific sentence-embedding model.

### Findings

**Decision**: 
The technology stack for the RAG backend will be:
- **API Framework**: FastAPI
- **Vector Database**: ChromaDB
- **Embedding Model**: Gemini Embedding Models (via Google AI SDK)

**Rationale**: 
- **FastAPI** was chosen for its high performance, ease of use, and automatic generation of interactive API documentation, which is ideal for this project.
- **ChromaDB** was selected because it is an open-source, lightweight, and easy-to-use vector database that integrates well with Python. This fits the user's requirement for a free and straightforward solution suitable for local development and initial deployment.
- **Gemini Embedding Models** were chosen as per the user's request, providing access to Google's latest embedding capabilities.

**Alternatives Considered**: 
- **API**: Flask was considered but FastAPI's performance and built-in async support made it a better choice.
- **Vector DB**: FAISS, Pinecone, and Weaviate were considered. FAISS is a library, not a full database, adding complexity. Pinecone is a managed service with potential costs. Weaviate is a powerful option but more complex to set up than ChromaDB for this project's scale.
- **Embedding Model**: Open-source models like `all-MiniLM-L6-v2` and `BAAI/bge-small-en-v1.5` were considered but the user explicitly chose to use Gemini models.