# Tasks: Integrated RAG Chatbot

**Input**: Design documents from `specs/001-rag-chatbot/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), data-model.md, contracts/

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **backend**: `backend/src/`, `backend/tests/`
- **frontend**: `frontend/src/`, `frontend/tests/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [X] T001 [P] Create backend project structure in `backend/`
- [X] T002 [P] Create frontend project structure in `frontend/`
- [X] T003 [P] Initialize backend Python project with FastAPI and other dependencies in `backend/`
- [X] T004 [P] Initialize frontend React project in `frontend/`
- [X] T005 [P] Configure linting and formatting tools for both projects

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T006 [P] Setup database schema and migrations framework for Neon in `backend/src/core/`
- [X] T007 [P] Implement user authentication/authorization framework using JWT in `backend/src/core/security.py`
- [X] T008 [P] Setup API routing and middleware structure in `backend/src/main.py`
- [X] T009 [P] Create base Pydantic models for all entities in `backend/src/models/`
- [X] T010 [P] Configure error handling and logging infrastructure for the backend in `backend/src/core/`
- [X] T011 [P] Setup environment configuration management for the backend in `backend/src/core/config.py`
- [X] T012 [P] Implement the embedding pipeline script in `backend/scripts/embed.py`

---

## Phase 3: User Story 1 - Ask a question about the entire book (Priority: P1) 🎯 MVP

**Goal**: A reader can ask a question about the entire book and receive a relevant, cited answer.

**Independent Test**: Can be tested by opening the chat widget, asking a question, and verifying that the answer is relevant, accurate, and includes citations.

### Implementation for User Story 1

- [X] T013 [P] [US1] Create the core chat widget component in `frontend/src/components/ChatWidget.js`
- [X] T014 [P] [US1] Implement state management for the chat widget in `frontend/src/`
- [X] T015 [P] [US1] Implement the API service for the `/query` endpoint in `frontend/src/services/api.js`
- [X] T016 [US1] Implement the `/query` endpoint in `backend/src/api/endpoints/query.py`
- [X] T017 [US1] Implement the RAG service in `backend/src/services/rag_service.py` to retrieve context from Qdrant and generate an answer from OpenAI.
- [X] T018 [US1] Integrate the chat widget with the Docusaurus site.

---

## Phase 4: User Story 2 - Ask a question about selected text (Priority: P2)

**Goal**: A reader can ask a question about a selected piece of text.

**Independent Test**: Can be tested by selecting text, clicking the "chat about this" button, and asking a question to verify the answer is only based on the selected context.

### Implementation for User Story 2

- [X] T019 [P] [US2] Implement the "chat about this" button functionality in the Docusaurus frontend.
- [X] T020 [P] [US2] Implement the API service for the `/selected-query` endpoint in `frontend/src/services/api.js`
- [X] T021 [US2] Implement the `/selected-query` endpoint in `backend/src/api/endpoints/query.py`
- [X] T022 [US2] Update the RAG service in `backend/src/services/rag_service.py` to handle selected text queries.

---

## Phase 5: User Story 3 - View Chat History (Priority: P3)

**Goal**: A reader can view their past conversations.

**Independent Test**: Can be tested by having a conversation, closing and reopening the chat widget, and verifying the conversation is in the history.

### Implementation for User Story 3

- [X] T023 [P] [US3] Implement the API service for the `/conversations` and `/conversations/{conversation_id}/messages` endpoints in `frontend/src/services/api.js`
- [X] T024 [P] [US3] Implement the `/conversations` and `/conversations/{conversation_id}/messages` endpoints in `backend/src/api/endpoints/history.py`
- [X] T025 [P] [US3] Implement the database services for retrieving chat history from Neon in `backend/src/services/history_service.py`
- [X] T026 [P] [US3] Implement the UI for displaying chat history in the chat widget.
- [X] T027 [P] [US3] Implement user-initiated deletion of chat history.

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [X] T028 [P] Documentation updates in `docs/`
- [X] T029 Code cleanup and refactoring across the backend and frontend.
- [X] T030 Performance optimization of the RAG pipeline.
- [X] T031 Security hardening of the backend API.
- [X] T032 Run `quickstart.md` validation.
- [X] T033 [P] Refactor frontend state management to provide chat context to all pages.

---

## Dependencies & Execution Order

-   **Setup (Phase 1)** must be completed before **Foundational (Phase 2)**.
-   **Foundational (Phase 2)** must be completed before any User Story phase.
-   User Story phases can be implemented in any order after Phase 2 is complete, but the priority order (P1 > P2 > P3) is recommended.
-   **Polish (Phase 6)** should be done after all desired User Story phases are complete.
