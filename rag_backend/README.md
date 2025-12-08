# ROS 2 Module RAG Backend

This directory contains the FastAPI application for the Retrieval-Augmented Generation (RAG) backend of the "Module 1: The Robotic Nervous System (ROS 2)" book.

To get started, follow the instructions in the main project's `quickstart.md` or directly:

1.  `cd rag_backend`
2.  `python -m venv .venv`
3.  `source .venv/bin/activate`
4.  `pip install -e .` (after `pyproject.toml` is set up correctly)
5.  `uvicorn src.api.main:app --reload`
