import os
import asyncio
import uuid
from rag_backend.src.services.content_loader import load_markdown_files
from rag_backend.src.services.text_splitter import TextSplitter
from rag_backend.src.services.embedding_service import EmbeddingService
from rag_backend.src.services.vector_store import VectorStore

async def ingest_book_content(
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
    embeddings = await embedding_service.get_embeddings(chunk_contents)
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
    
    # Create a dummy docs directory and a markdown file for testing
    current_dir = os.path.dirname(__file__)
    project_root = os.path.abspath(os.path.join(current_dir, "..", "..")) # Project root for ingest.py
    book_frontend_docs_path = os.path.join(project_root, "book_frontend", "docs")
    chroma_db_path = os.path.join(project_root, "chroma_db")

    os.makedirs(book_frontend_docs_path, exist_ok=True)
    with open(os.path.join(book_frontend_docs_path, "dummy_chapter.md"), "w", encoding='utf-8') as f:
        f.write("# Dummy Chapter\n\nThis is some dummy content for testing the ingestion script. It has multiple sentences to ensure chunking works.")

    # Set a dummy API key for local testing (replace with actual key or ensure env var is set)
    os.environ["GEMINI_API_KEY"] = os.getenv("GEMINI_API_KEY", "dummy_key_for_testing")

    # Clean up previous chroma_db if exists
    if os.path.exists(chroma_db_path):
        import shutil
        shutil.rmtree(chroma_db_path)

    asyncio.run(ingest_book_content(
        docs_path=book_frontend_docs_path,
        chroma_persist_dir=chroma_db_path
    ))

    # Clean up dummy files
    os.remove(os.path.join(book_frontend_docs_path, "dummy_chapter.md"))
    os.rmdir(book_frontend_docs_path) # Only removes if empty
    # shutil.rmtree(chroma_db_path) # Optionally clean up chroma db after test
