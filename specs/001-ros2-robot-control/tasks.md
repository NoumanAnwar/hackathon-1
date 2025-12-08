---
description: "Task list for feature 'Module 1: The Robotic Nervous System (ROS 2)'"
---

# Tasks: Module 1: The Robotic Nervous System (ROS 2)

**Input**: Design documents from `specs/001-ros2-robot-control/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [X] T001 Create main project directories: `book_frontend/` and `rag_backend/`
- [X] T002 [P] Initialize the Docusaurus project by creating `book_frontend/package.json` and `book_frontend/docusaurus.config.js`
- [X] T003 [P] Initialize the FastAPI project by creating `rag_backend/pyproject.toml` and `rag_backend/src/api/main.py`
- [X] T004 [P] Create placeholder `README.md` files for `book_frontend/` and `rag_backend/`

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure for the RAG backend.

**⚠️ CRITICAL**: No user story work can begin until this phase is complete.

- [X] T005 Define Pydantic models for API request/response in `rag_backend/src/models/api_models.py`
- [X] T006 Implement a script to traverse and read markdown files from `book_frontend/docs/` in `rag_backend/src/services/content_loader.py`
- [X] T007 [P] Implement a text chunking service in `rag_backend/src/services/text_splitter.py`
- [X] T008 [P] Implement a service to generate embeddings using the Gemini API in `rag_backend/src/services/embedding_service.py`
- [X] T009 Implement a service to store and query embeddings in ChromaDB in `rag_backend/src/services/vector_store.py`
- [X] T010 Create a main ingestion script `rag_backend/ingest.py` that uses services T006-T009 to process and store book content.
- [X] T011 Implement the basic `/api/query` endpoint in `rag_backend/src/api/main.py` to accept queries and return a mocked response.

**Checkpoint**: Foundation ready - RAG backend can be started and accepts queries. Content writing can begin.

---

## Phase 3: User Story 1 - Understand Core ROS 2 Concepts (Priority: P1) 🎯 MVP

**Goal**: Deliver the first chapter of the book explaining core ROS 2 concepts.

**Independent Test**: The Docusaurus site can be built, and the content of Chapter 1 is rendered correctly and is understandable.

### Implementation for User Story 1

- [X] T012 [US1] Write the content for "Chapter 1: ROS 2 Graph Basics" in `book_frontend/docs/chapter1.md`

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently. The book has its first chapter.

---

## Phase 4: User Story 2 - Run a Practical Python Example (Priority: P2)

**Goal**: Deliver the second chapter, including a runnable Python publisher/subscriber example.

**Independent Test**: A user can follow the instructions in Chapter 2 to run the Python example and see the nodes communicating.

### Implementation for User Story 2

- [X] T013 [US2] Create a new ROS 2 package `ros2_examples` with a `package.xml` and `setup.py`.
- [X] T014 [P] [US2] Implement the 'talker' (publisher) Python node in `ros2_examples/ros2_examples/publisher_node.py`
- [X] T015 [P] [US2] Implement the 'listener' (subscriber) Python node in `ros2_examples/ros2_examples/subscriber_node.py`
- [X] T016 [US2] Create a launch file to run both nodes in `ros2_examples/launch/example.launch.py`
- [X] T017 [US2] Write the content for "Chapter 2: Python Agents with rclpy" in `book_frontend/docs/chapter2.md`, including code blocks and instructions.

**Checkpoint**: At this point, User Story 2 content and code are complete and testable.

---

## Phase 5: User Story 3 - Model a Simple Robot (Priority: P3)

**Goal**: Deliver the third chapter, explaining URDFs with a simple, visualizable example.

**Independent Test**: A user can follow the instructions in Chapter 3 to visualize the provided URDF file using RViz2.

### Implementation for User Story 3

- [X] T018 [P] [US3] Create a simple URDF file for a two-link robot arm in `book_frontend/static/urdf/simple_arm.urdf`
- [X] T019 [P] [US3] Create a launch file to visualize the URDF in RViz2 in `book_frontend/static/urdf/display.launch.py`
- [X] T020 [US3] Write the content for "Chapter 3: URDF for Humanoids" in `book_frontend/docs/chapter3.md`, explaining the URDF structure.

**Checkpoint**: All user stories for the core book content are now complete.

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Integrate the RAG backend with the frontend and finalize the project.

- [X] T021 [P] Implement the full RAG logic in the `/api/query` endpoint in `rag_backend/src/api/main.py` to generate real answers.
- [X] T022 [P] Create a React component for a chat interface in `book_frontend/src/components/Chatbox.js`
- [X] T023 Integrate the Chatbox component with the Docusaurus site, connecting it to the FastAPI backend API.
- [X] T024 Configure the main Docusaurus navigation and sidebar for all chapters in `book_frontend/docusaurus.config.js`
- [X] T025 Create the main `README.md` for the project root, explaining the structure and how to run both frontend and backend.

---

## Dependencies & Execution Order

- **Setup (Phase 1)** must complete before all other phases.
- **Foundational (Phase 2)** depends on Setup. It blocks all User Story and Polish phases.
- **User Stories (Phases 3-5)** can be worked on in parallel after the Foundational phase is complete.
- **Polish (Phase 6)** depends on the completion of the RAG backend (T021) and the Docusaurus setup (T002).

## Implementation Strategy

The suggested approach is **MVP First**.
1. Complete Phase 1 (Setup) and Phase 2 (Foundational).
2. Complete Phase 3 (User Story 1). At this point, a functional book with one chapter exists.
3. Incrementally add User Stories 2 and 3.
4. Complete Phase 6 to integrate the RAG chatbot functionality.
