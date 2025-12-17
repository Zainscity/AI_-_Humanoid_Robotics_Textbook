# Quickstart: Integrated RAG Chatbot

This guide provides instructions for setting up the development environment for the Integrated RAG Chatbot feature.

## Prerequisites

-   Python 3.11+
-   Node.js 18+
-   Docker and Docker Compose (Optional, for local PostgreSQL setup if not using Neon)

## Backend Setup

1.  **Navigate to the backend directory:**
    ```bash
    cd backend
    ```

2.  **Create a virtual environment:**
    ```bash
    python3 -m venv .venv
    ```

3.  **Activate the virtual environment:**
    ```bash
    source .venv/bin/activate
    ```

4.  **Install Python dependencies:**
    ```bash
    pip install -r requirements.txt
    ```

5.  **Set up environment variables:**
    Create a `.env` file in the `backend` directory and add the following variables. Replace placeholder values with your actual credentials.
    ```
    DATABASE_URL=<your-neon-database-url>
    QDRANT_URL=<your-qdrant-cloud-url>
    QDRANT_API_KEY=<your-qdrant-api-key>
    OPENAI_API_KEY=<your-openai-api-key>
    GEMINI_API_KEY=<your-gemini-api-key>
    SECRET_KEY=<a-long-random-string-for-jwt>
    ALGORITHM=HS256
    ACCESS_TOKEN_EXPIRE_MINUTES=30
    ```

6.  **Run database migrations:**
    ```bash
    alembic upgrade head
    ```

7.  **Run the backend server:**
    ```bash
    uvicorn src.main:app --reload
    ```

## Frontend Setup

1.  **Navigate to the project root:**
    ```bash
    cd ..
    ```

2.  **Install Node.js dependencies:**
    ```bash
    npm install
    ```

3.  **Run the Docusaurus development server:**
    ```bash
    npm start
    ```

## Embedding Pipeline

To populate the Qdrant vector database with the book's content, run the embedding pipeline:
```bash
cd backend
source .venv/bin/activate
python3 scripts/embed.py
```
This script will read the MDX files from the `docs` directory, chunk them, generate embeddings using the OpenAI API, and upload them to Qdrant.

## Authentication and Usage

-   **Registering a User**: Currently, user registration is not implemented in the API. You would typically have an endpoint like `/auth/register` for this.
-   **Logging In**: Use the `/auth/token` endpoint with username (email) and password to get a JWT.
-   **Making Authenticated Requests**: Include the JWT in the `Authorization: Bearer <token>` header for endpoints requiring authentication.