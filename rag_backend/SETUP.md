# RAG Backend Setup Guide

This guide will help you set up and run the RAG (Retrieval-Augmented Generation) backend for the ROS 2 book.

## Prerequisites

1. A Google AI Studio account to get a Gemini API key
2. Python 3.8 or higher installed

## Getting Started

### 1. Get Your Gemini API Key

1. Go to [Google AI Studio](https://aistudio.google.com/)
2. Create an account or sign in
3. Create a new API key
4. Copy the API key

### 2. Set Up Environment Variables

Create a `.env` file in the `rag_backend` directory (or use the existing one) and add your API key:

```bash
GEMINI_API_KEY=your_actual_api_key_here
```

Replace `your_actual_api_key_here` with the API key you obtained from Google AI Studio.

### 3. Install Dependencies

Navigate to the rag_backend directory and install the package:

```bash
cd rag_backend
pip install -e .
```

### 4. Run the Ingestion Script

Before starting the API server, you need to index your book content:

```bash
cd rag_backend
python -m rag_backend.ingest
```

This will load all markdown files from the `book_frontend/docs` directory, chunk them, generate embeddings, and store them in ChromaDB.

### 5. Start the API Server

Start the FastAPI server:

```bash
cd rag_backend
uvicorn src.api.main:app --reload
```

The API will be available at `http://localhost:8000`.

## API Endpoints

- `GET /` - Health check endpoint
- `POST /api/query` - Query endpoint for RAG responses

Example query body:
```json
{
    "query": "What are ROS 2 nodes?"
}
```

## Troubleshooting

1. **"GEMINI_API_KEY is not set" Error:** Ensure your `.env` file has the correct API key and it's in the correct location.

2. **Ingestion Fails:** Make sure you have content in the `book_frontend/docs` directory before running the ingestion script.

3. **API Returns Generic Response:** This may indicate that no relevant content was found in the vector store - make sure the ingestion completed successfully.

## Running with Docker

If you prefer Docker, you can build and run the backend in a container:

```bash
# Build the image
docker build -t rag_backend .

# Run the container
docker run -p 8000:8000 -e GEMINI_API_KEY=your_api_key_here rag_backend
```