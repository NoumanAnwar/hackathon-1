import os
import chromadb
from typing import List, Dict, Union
from chromadb.utils import embedding_functions

# A placeholder embedding function for ChromaDB initialization.
# In a real scenario, this would be the Gemini embedding function.
class PlaceholderEmbeddingFunction(embedding_functions.EmbeddingFunction):
    def __call__(self, texts: List[str]) -> List[List[float]]:
        # This is a dummy implementation. Replace with actual Gemini embedding.
        # Ensure the length matches the expected embedding dimension (e.g., 768 for small models)
        return [[0.0] * 768 for _ in texts] # Return list of dummy embeddings

class VectorStore:
    """
    Service to store and query embeddings using ChromaDB.
    """
    def __init__(self, collection_name: str = "book_rag_collection", persist_directory: str = "./chroma_db"):
        self.persist_directory = persist_directory
        self.client = chromadb.PersistentClient(path=self.persist_directory)
        
        # Initialize with a dummy embedding function for now.
        # This will be replaced with the actual Gemini embedding function during ingestion.
        self.collection = self.client.get_or_create_collection(
            name=collection_name,
            embedding_function=PlaceholderEmbeddingFunction() # Will be dynamically set during ingestion
        )

    def add_documents(self, documents: List[Dict[str, Union[str, List[float]]]]) -> None:
        """
        Adds documents (chunks with embeddings) to the ChromaDB collection.
        Documents should be in the format:
        [{'id': 'doc_id', 'content': 'text', 'embedding': [0.1, 0.2], 'metadata': { 'source_path': '...'}}]
        """
        ids = [doc['id'] for doc in documents]
        embeddings = [doc['embedding'] for doc in documents]
        metadatas = [doc['metadata'] for doc in documents]
        documents_content = [doc['content'] for doc in documents]

        # ChromaDB expects separate lists for documents, embeddings, metadatas, and ids
        self.collection.add(
            embeddings=embeddings,
            metadatas=metadatas,
            documents=documents_content,
            ids=ids
        )
        print(f"Added {len(documents)} documents to ChromaDB collection '{self.collection.name}'.")

    def query_documents(self, query_embedding: List[float], n_results: int = 5) -> List[Dict]:
        """
        Queries the ChromaDB collection with an embedding and returns the most relevant documents.
        """
        results = self.collection.query(
            query_embeddings=[query_embedding],
            n_results=n_results,
            include=['documents', 'distances', 'metadatas']
        )
        # Format results into a more usable list of dictionaries
        formatted_results = []
        if results and results['documents']:
            for i in range(len(results['documents'][0])):
                formatted_results.append({
                    "content": results['documents'][0][i],
                    "distance": results['distances'][0][i],
                    "metadata": results['metadatas'][0][i]
                })
        return formatted_results

if __name__ == "__main__":
    # Example usage
    persist_dir = "./test_chroma_db"
    if os.path.exists(persist_dir):
        import shutil
        shutil.rmtree(persist_dir) # Clear previous test data

    vector_store = VectorStore(collection_name="test_collection", persist_directory=persist_dir)

    # Dummy documents with placeholder embeddings
    dummy_documents = [
        {
            "id": "doc1",
            "content": "ROS 2 is a flexible framework for writing robot software.",
            "embedding": [0.1] * 768, # Placeholder
            "metadata": {"source_path": "chapter1.md", "page_number": 1}
        },
        {
            "id": "doc2",
            "content": "Nodes are fundamental components in the ROS 2 graph.",
            "embedding": [0.2] * 768, # Placeholder
            "metadata": {"source_path": "chapter1.md", "page_number": 2}
        },
        {
            "id": "doc3",
            "content": "Python with rclpy is used for developing ROS 2 applications.",
            "embedding": [0.3] * 768, # Placeholder
            "metadata": {"source_path": "chapter2.md", "page_number": 1}
        },
    ]

    vector_store.add_documents(dummy_documents)

    # Dummy query embedding
    query_emb = [0.15] * 768 # Placeholder, should be embedding of a query
    results = vector_store.query_documents(query_emb, n_results=2)

    print("\nQuery Results:")
    for res in results:
        print(f"Content: {res['content'][:50]}...")
        print(f"Distance: {res['distance']}")
        print(f"Source: {res['metadata']['source_path']}\n")

    # Clean up test directory
    shutil.rmtree(persist_dir)
