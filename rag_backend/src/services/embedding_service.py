import os
import google.generativeai as genai
from typing import List, Dict, Union

class EmbeddingService:
    """
    Service to generate embeddings using the Gemini API.
    Requires GEMINI_API_KEY to be set as an environment variable.
    """
    def __init__(self, model_name: str = "models/embedding-001"):
        self.model_name = model_name
        api_key = os.getenv("GEMINI_API_KEY")
        if not api_key:
            raise ValueError("GEMINI_API_KEY environment variable not set.")
        genai.configure(api_key=api_key)

    async def get_embedding(self, text: str) -> List[float]:
        """
        Generates an embedding for a single piece of text.
        """
        if not text:
            return []
        
        # The embedding API expects a list of texts, even for a single text.
        # It also requires the `task_type` parameter.
        result = await genai.embed_content(
            model=self.model_name,
            content=[text],
            task_type="RETRIEVAL_DOCUMENT" # or RETRIEVAL_QUERY depending on usage
        )
        # The result contains embeddings for all texts in the input list.
        # We only sent one, so we take the first one.
        return result['embedding'][0]

    async def get_embeddings(self, texts: List[str]) -> List[List[float]]:
        """
        Generates embeddings for a list of texts.
        """
        if not texts:
            return []
        
        result = await genai.embed_content(
            model=self.model_name,
            content=texts,
            task_type="RETRIEVAL_DOCUMENT"
        )
        return result['embedding']

if __name__ == "__main__":
    # Example usage
    # Ensure GEMINI_API_KEY is set in your environment
    
    # This needs to be run in an async context, for example, using asyncio
    import asyncio

    async def main():
        os.environ["GEMINI_API_KEY"] = "YOUR_GEMINI_API_KEY" # Replace with a dummy key for testing

        try:
            embedding_service = EmbeddingService()
            
            # Test single embedding
            text1 = "What is a ROS 2 node?"
            embedding1 = await embedding_service.get_embedding(text1)
            print(f"Embedding for '{text1[:20]}...': Length {len(embedding1)}, First 5: {embedding1[:5]}")

            # Test multiple embeddings
            texts = [
                "ROS 2 topics are a communication mechanism.",
                "URDF is used for robot description."
            ]
            embeddings = await embedding_service.get_embeddings(texts)
            for i, emb in enumerate(embeddings):
                print(f"Embedding for '{texts[i][:20]}...': Length {len(emb)}, First 5: {emb[:5]}")

        except ValueError as e:
            print(f"Error: {e}. Please set the GEMINI_API_KEY environment variable.")
        except Exception as e:
            print(f"An unexpected error occurred: {e}")

    asyncio.run(main())
