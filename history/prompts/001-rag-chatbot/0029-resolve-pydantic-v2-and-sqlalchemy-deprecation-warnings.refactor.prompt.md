---
id: 0029
title: Resolve Pydantic V2 and SQLAlchemy Deprecation Warnings
stage: refactor
date: 2026-05-26
surface: agent
model: gemini-2.0-flash-exp
feature: 001-rag-chatbot
branch: main
user: zainscity
command: none
labels: ["pydantic", "sqlalchemy", "refactor"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - backend/src/models/schemas.py
 - backend/src/core/config.py
 - backend/src/core/database.py
tests:
 - pytest (collection verified warnings gone)
---

## Prompt

I am getting this error in backend logs (2026-05-26 02:34:22.177 [error] /var/task/_vendor/pydantic/_internal/_config.py:341: UserWarning: Valid config keys have changed in V2:
* 'orm_mode' has been renamed to 'from_attributes'
  warnings.warn(message, UserWarning))

## Response snapshot

Resolved the Pydantic V2 warning by renaming 'orm_mode' to 'from_attributes' in 'backend/src/models/schemas.py'. Also updated 'backend/src/core/config.py' and 'backend/src/core/database.py' to address other Pydantic and SQLAlchemy deprecation warnings detected during verification.

## Outcome

- ✅ Impact: Cleaned up backend logs and ensured compatibility with Pydantic V2 and SQLAlchemy 2.0.
- 🧪 Tests: Verified resolution via pytest (warnings no longer appear in output).
- 📁 Files: Updated schemas.py, config.py, and database.py.
- 🔁 Next prompts: null
- 🧠 Reflection: Proactively addressing related deprecations maintains codebase health.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
