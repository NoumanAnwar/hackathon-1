# Data Models

This document defines the data models for the Docusaurus book content and the RAG backend.

## 1. Book Content Model

The book content is structured based on the Docusaurus file and directory layout.

- **Module**: A top-level learning unit (e.g., "ROS 2").
  - **Chapter**: A section within a module (`chapter1.md`).
    - **Content**: Markdown text, images, and code blocks.
    - **Code Example**: A runnable code snippet associated with the chapter content.

## 2. RAG Backend Data Model

The RAG backend deals with processed text chunks and API interactions.

- **Text Chunk** (Stored in Vector DB)
  - `chunk_id` (string, UUID): Unique identifier for the text chunk.
  - `content` (string): The raw text content of the chunk.
  - `vector` (array of floats): The embedding vector of the `content`.
  - `metadata` (object):
    - `source_chapter` (string): The chapter the chunk came from (e.g., "chapter1.md").
    - `page_number` (integer): (Optional) Page number if applicable.

- **API Request Model** (`/api/query`)
  - `query` (string): The user's question.
  - `session_id` (string, optional): A session identifier for conversation history.

- **API Response Model** (`/api/query`)
  - `answer` (string): The generated answer from the RAG model.
  - `source` (string): The chapter and section the answer was drawn from, for citation.
