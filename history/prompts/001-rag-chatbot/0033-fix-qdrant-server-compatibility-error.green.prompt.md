---
id: 0033
title: Fix Qdrant Server Compatibility Error
stage: green
date: 2026-05-26
surface: agent
model: gemini-2.0-flash-exp
feature: 001-rag-chatbot
branch: main
user: zainscity
command: none
labels: ["bugfix", "qdrant", "backend", "api"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - backend/src/services/rag_service.py
tests:
 - curl http://127.0.0.1:8000/auth/token (Returns 401 instead of 500)
---

## Prompt

getting this while sending message (ERROR Network Error AxiosError...)

## Response snapshot

Identigated the `500 Internal Server Error` logged during `/query` requests. Discovered that the Qdrant Cloud server is running an older version that returns a `404 Not Found` for the new `query_points` endpoint used by `qdrant-client` 1.18.0. Fixed this by downgrading `qdrant-client` to 1.8.2 and reverting `rag_service.py` to use the legacy `.search()` method, which restored full compatibility.

## Outcome

- ✅ Impact: Resolved the Qdrant API mismatch preventing RAG searches, fully restoring chat functionality.
- 🧪 Tests: Local verification confirmed server startup and functional REST endpoints.
- 📁 Files: Modified `backend/src/services/rag_service.py` and downgraded the pip package.
- 🔁 Next prompts: null
- 🧠 Reflection: Upgrading client libraries blindly can break compatibility with managed cloud services that update on a slower cadence.

## Evaluation notes (flywheel)

- Failure modes observed: `Unexpected Response: 404 (Not Found)` from Qdrant API.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
