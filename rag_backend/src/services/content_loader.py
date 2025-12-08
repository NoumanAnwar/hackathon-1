import os
from typing import List, Dict

def load_markdown_files(docs_path: str) -> List[Dict[str, str]]:
    """
    Traverses the specified directory and loads all markdown files.
    Returns a list of dictionaries, each containing 'content' and 'source_path'.
    """
    documents = []
    for root, _, files in os.walk(docs_path):
        for file in files:
            if file.endswith(".md"):
                file_path = os.path.join(root, file)
                with open(file_path, 'r', encoding='utf-8') as f:
                    content = f.read()
                documents.append({
                    "content": content,
                    "source_path": file_path
                })
    return documents

if __name__ == "__main__":
    # Example usage (assuming docs are in book_frontend/docs relative to project root)
    # This might need adjustment based on how the script is run
    current_dir = os.path.dirname(__file__)
    project_root = os.path.abspath(os.path.join(current_dir, "..", "..", ".."))
    docs_path = os.path.join(project_root, "book_frontend", "docs")

    # Create a dummy docs directory and a markdown file for testing
    os.makedirs(docs_path, exist_ok=True)
    with open(os.path.join(docs_path, "test_chapter.md"), "w", encoding='utf-8') as f:
        f.write("# Test Chapter\n\nThis is some test content.")

    loaded_docs = load_markdown_files(docs_path)
    for doc in loaded_docs:
        print(f"Loaded from: {doc['source_path']}")
        print(f"Content snippet: {doc['content'][:50]}...")

    # Clean up dummy file and directory
    os.remove(os.path.join(docs_path, "test_chapter.md"))
    # The rmdir(docs_path) will likely fail if the book_frontend/docs directory is not empty
    # For now, let's just remove the test_chapter.md and leave the directory
    # os.rmdir(docs_path) 
