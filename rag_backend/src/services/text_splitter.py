from typing import List, Dict

class TextSplitter:
    """
    A simple text splitter to break down documents into smaller, manageable chunks.
    """
    def __init__(self, chunk_size: int = 500, chunk_overlap: int = 50):
        self.chunk_size = chunk_size
        self.chunk_overlap = chunk_overlap

    def split_text(self, text: str, source_path: str = "unknown_source") -> List[Dict[str, str]]:
        """
        Splits a given text into chunks.
        """
        if not text:
            return []

        words = text.split()
        chunks = []
        for i in range(0, len(words), self.chunk_size - self.chunk_overlap):
            chunk_words = words[i : i + self.chunk_size]
            chunk_content = " ".join(chunk_words)
            chunks.append({
                "content": chunk_content,
                "source_path": source_path,
                "chunk_start_index": i # Can be used to reconstruct original position
            })
        return chunks

if __name__ == "__main__":
    splitter = TextSplitter(chunk_size=10, chunk_overlap=2)
    sample_text = (
        "This is a sample document that needs to be split into smaller chunks. "
        "Each chunk should have a certain size, and there can be some overlap "
        "between consecutive chunks to preserve context across splits. "
        "This helps ensure that no crucial information is lost at the boundaries."
    )
    
    chunks = splitter.split_text(sample_text, source_path="sample.md")
    for i, chunk in enumerate(chunks):
        print(f"Chunk {i+1} (Source: {chunk['source_path']}, Start Index: {chunk['chunk_start_index']}):")
        print(f"  '{chunk['content']}'\n")
