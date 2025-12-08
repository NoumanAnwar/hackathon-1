# AI/Spec Book + RAG Chatbot on Physical AI & Humanoid Robotics

This repository contains the code and content for "Module 1: The Robotic Nervous System (ROS 2)", an educational resource for engineering students. It features a Docusaurus-powered book and an integrated RAG (Retrieval-Augmented Generation) chatbot.

## Project Structure

-   `book_frontend/`: Contains the Docusaurus project for the online book.
    -   `book_frontend/docs/`: Markdown content for the book chapters.
    -   `book_frontend/src/`: React components and Docusaurus configuration.
-   `rag_backend/`: Contains the FastAPI application for the RAG chatbot backend.
    -   `rag_backend/src/api/`: FastAPI main application and API endpoints.
    -   `rag_backend/src/services/`: Core RAG services (content loading, embedding, vector store).
    -   `rag_backend/src/models/`: Pydantic models for API requests/responses.
-   `ros2_examples/`: Contains the ROS 2 Python example packages.
    -   `ros2_examples/ros2_examples/`: Python nodes (publisher, subscriber).
    -   `ros2_examples/launch/`: ROS 2 launch files.
-   `specs/`: Project specifications, plans, and task lists.
-   `history/`: Prompt history records.

## Quickstart

Follow these steps to get the project running locally. Ensure you have Node.js (v18+), Python (3.9+), and ROS 2 Humble Hawksbill installed.

### 1. Set up API Key

Ensure you have a `GEMINI_API_KEY` environment variable set with your Google AI API key. This is required for the RAG backend.

```bash
export GEMINI_API_KEY="YOUR_GEMINI_API_KEY"
```
*(On Windows, use `set GEMINI_API_KEY="YOUR_GEMINI_API_KEY"`)*

### 2. Book Frontend (Docusaurus)

```bash
cd book_frontend
npm install
npm start
```
The Docusaurus site will be available at `http://localhost:3000`.

### 3. RAG Backend (FastAPI)

```bash
cd rag_backend
python -m venv .venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate
pip install -e .           # Install the backend as an editable package
uvicorn src.api.main:app --reload
```
The FastAPI service will be available at `http://localhost:8000`, with interactive documentation at `http://localhost:8000/docs`.

### 4. Ingest Book Content for RAG

Before using the RAG chatbot, you need to ingest the book's content into the vector database:

```bash
cd rag_backend
source .venv/bin/activate # (if not already active)
python ingest.py
```

### 5. ROS 2 Examples

To build and run the ROS 2 examples:

```bash
# In your project root
# Create a colcon workspace (if not already done for other ROS 2 work)
mkdir -p ros2_ws/src
ln -s ../ros2_examples ros2_ws/src/ros2_examples # Symlink your package

cd ros2_ws
source /opt/ros/humble/setup.bash # Source your ROS 2 environment
colcon build --packages-select ros2_examples
source install/setup.bash # Source your workspace

# Now launch the example
ros2 launch ros2_examples example.launch.py
```

## Contributing

Please refer to the `specs/` directory for detailed specifications and plans.
