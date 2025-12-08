# Quickstart Guide

This guide explains how to set up the development environment to work on this project.

## Prerequisites

- Node.js v18+
- Python 3.9+
- ROS 2 Humble Hawksbill
- An environment variable `GEMINI_API_KEY` set with your Google AI API key.

## 1. Book Frontend (Docusaurus)

The book content is in the `book_frontend` directory.

1.  **Navigate to the directory**:
    ```bash
    cd book_frontend
    ```

2.  **Install dependencies**:
    ```bash
    npm install
    ```

3.  **Run the development server**:
    ```bash
    npm start
    ```

The Docusaurus site will be available at `http://localhost:3000`.

## 2. RAG Backend (FastAPI)

The RAG backend service is in the `rag_backend` directory.

1.  **Navigate to the directory**:
    ```bash
    cd rag_backend
    ```

2.  **Create a virtual environment**:
    ```bash
    python -m venv .venv
    source .venv/bin/activate
    ```

3.  **Install Python dependencies**:
    ```bash
    pip install -r requirements.txt
    ```
    *(Note: The `requirements.txt` file will be created in a later development task).*

4.  **Run the development server**:
    ```bash
    uvicorn src.api.main:app --reload
    ```

The FastAPI service will be available at `http://localhost:8000`, with interactive documentation at `http://localhost:8000/docs`.

## 3. ROS 2 Examples

The ROS 2 code examples are designed to be run within a ROS 2 workspace.

1.  **Source your ROS 2 environment**:
    ```bash
    source /opt/ros/humble/setup.bash
    ```

2.  **Run the example nodes**:
    *(Specific instructions will be provided within each chapter of the book).*
