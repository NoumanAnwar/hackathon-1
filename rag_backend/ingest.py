import os
import uuid
from src.services.content_loader import load_markdown_files
from src.services.text_splitter import TextSplitter
from src.services.embedding_service import EmbeddingService
from src.services.vector_store import VectorStore

def ingest_book_content(
    docs_path: str,
    chroma_persist_dir: str = "./chroma_db",
    collection_name: str = "book_rag_collection"
):
    print(f"Starting content ingestion from: {docs_path}")

    # 1. Load markdown files
    documents = load_markdown_files(docs_path)
    print(f"Loaded {len(documents)} markdown files.")

    # 2. Split text into chunks
    splitter = TextSplitter(chunk_size=500, chunk_overlap=50) # Use default chunk size/overlap
    all_chunks = []
    for doc in documents:
        chunks = splitter.split_text(doc["content"], source_path=doc["source_path"])
        all_chunks.extend(chunks)
    print(f"Split into {len(all_chunks)} text chunks.")

    # 3. Generate embeddings
    embedding_service = EmbeddingService()
    # Extract just the content for embedding
    chunk_contents = [chunk["content"] for chunk in all_chunks]
    embeddings = embedding_service.get_embeddings(chunk_contents)
    print(f"Generated {len(embeddings)} embeddings.")

    # 4. Prepare documents for VectorStore
    chroma_documents = []
    for i, chunk in enumerate(all_chunks):
        chroma_documents.append({
            "id": str(uuid.uuid4()), # Generate a unique ID for each chunk
            "content": chunk["content"],
            "embedding": embeddings[i],
            "metadata": {
                "source_path": chunk["source_path"],
                "chunk_start_index": chunk["chunk_start_index"]
                # Add other metadata if available, e.g., chapter title, page_number
            }
        })

    # 5. Store in ChromaDB
    vector_store = VectorStore(collection_name=collection_name, persist_directory=chroma_persist_dir)
    vector_store.add_documents(chroma_documents)
    print("Content ingestion complete.")

if __name__ == "__main__":
    # Example usage:
    # Ensure GEMINI_API_KEY is set in your environment

    # Set a real API key or ensure env var is set
    if not os.getenv("GEMINI_API_KEY"):
        print("Warning: GEMINI_API_KEY environment variable is not set.")
        print("Please set it before running ingestion: export GEMINI_API_KEY=set your gemini api key")
        exit(1)

    # Define paths
    current_dir = os.path.dirname(__file__)
    project_root = os.path.abspath(os.path.join(current_dir, "..")) # Project root for rag_backend
    book_frontend_docs_path = os.path.join(project_root, "book_frontend", "docs")
    chroma_db_path = os.path.join(project_root, "chroma_db")

    # Make sure the docs directory exists
    if not os.path.exists(book_frontend_docs_path):
        print(f"Error: Docs directory does not exist at {book_frontend_docs_path}")
        exit(1)

    # Clean up previous chroma_db if exists
    if os.path.exists(chroma_db_path):
        import shutil
        shutil.rmtree(chroma_db_path)

    ingest_book_content(
        docs_path=book_frontend_docs_path,
        chroma_persist_dir=chroma_db_path
    )
