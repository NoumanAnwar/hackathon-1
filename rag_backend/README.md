# ROS 2 Module RAG Backend

This directory contains the FastAPI application for the Retrieval-Augmented Generation (RAG) backend of the "Module 1: The Robotic Nervous System (ROS 2)" book.

## Prerequisites

- Python 3.8+
- Google Gemini API Key (get it from [Google AI Studio](https://aistudio.google.com/))

## Setup Instructions

See the [SETUP.md](SETUP.md) file for complete setup instructions, including:

1. Setting up your Google Gemini API key
2. Installing dependencies
3. Running the ingestion script to index book content
4. Starting the API server

## Quick Start

After setting up your API key:

1. `cd rag_backend`
2. `pip install -e .`
3. `python ingest.py` (to index the book content)
4. `uvicorn src.api.main:app --reload` (to start the server)
