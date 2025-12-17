---
id: 0002
title: Update project constitution with new principles
stage: constitution
date: 2025-12-09
surface: agent
model: gemini-pro
feature: none
branch: main
user: zainscity
command: /sp.constitution
labels: [constitution, documentation]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - none
---

## Prompt

/sp.constitution CONSTITUTION (applies to ALL system components):
  ✅ "All architectural claims must be grounded in validated technical sources"
  ✅ "All AI workflows must follow safety, reliability, and reproducibility principles"
  ✅ "All descriptions must reference official documentation when defining tools or APIs"
  ✅ "APA-style citations required for any external source referenced"
  ✅ "Zero tolerance for unverifiable technical assumptions"
  ✅ "No hallucinated features, libraries, or capabilities may be included"
  ✅ "All data storage, processing, and API usage must comply with open-source licenses and provider policies"
  ✅ "All system components must support testability, observability, and maintainability"
  ✅ "Security best practices must be followed: no hard-coded secrets, secure environment variables, restricted API keys"
  ✅ "RAG system output must be grounded strictly in retrieved context, with verifiable traceability"

SPECIFICATION (applies only to THIS requirement: Integrated RAG Chatbot):
  ✅ "Build a fully integrated RAG (Retrieval-Augmented Generation) chatbot embedded inside the published Docusaurus book"
  ✅ "Chatbot must support two modes: (1) full-book RAG search; (2) selected-text-only answering"
  ✅ "Backend must use FastAPI, Qdrant Cloud, Neon Serverless PostgreSQL, and OpenAI Agents/ChatKit SDKs"
  ✅ "Frontend must embed a React-based chat widget inside Docusaurus with optional text-selection pipeline"
  ✅ "Embedding pipeline: chunk MDX → generate embeddings → upload to Qdrant"
  ✅ "Chatbot must cite source sections in responses and avoid hallucinations"
  ✅ "All data retrieval must be deterministic and traceable through metadata"
  ✅ "System must store chat history, metadata, and logs in Neon for auditability"
  ✅ "Performance target: sub-2 second retrieval latency for Qdrant queries"
  ✅ "Deployment target: fully functional chatbot published with the book (Version 2.0)"
  ✅ "Testing requirement: unit tests, integration tests, RAG validation tests, and fallback scenarios"
  ✅ "No modification to existing book content, theme files, layouts, or prior functionalities unless explicitly required for embedding the chatbot."
  ✅ "All enhancements must be additive and non-destructive to the existing Docusaurus project."

## Response snapshot

I have updated the constitution in `.specify/memory/constitution.md`.

## Outcome

- ✅ Impact: Updated the project constitution with 10 new principles.
- 🧪 Tests: None
- 📁 Files: 
  - `.specify/memory/constitution.md`
- 🔁 Next prompts: None
- 🧠 Reflection: The `create-phr.sh` script had a collision with an existing PHR ID. I had to manually create the PHR file with the correct ID.
