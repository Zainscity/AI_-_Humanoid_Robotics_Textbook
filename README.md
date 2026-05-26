# Physical AI & Humanoid Robotics: The Interactive Book

Welcome to the repository for **"Physical AI & Humanoid Robotics"**, an interactive, open-source web book designed to guide you through designing, building, and programming the next generation of intelligent, embodied agents.

This project is not just a static book; it features a **fully integrated RAG (Retrieval-Augmented Generation) Chatbot** that allows readers to converse with the textbook's content, ask complex questions, and get instant, context-aware answers powered by Google's Gemini models.

![Humanoid Robotics Banner](static/img/hero-background-dark.png)

## 🌟 Key Features

*   **Interactive Web Book:** Built with [Docusaurus](https://docusaurus.io/), providing a beautiful, fast, and accessible reading experience.
*   **AI RAG Chatbot:** A custom-built AI assistant that can answer questions based *strictly* on the book's content.
*   **Full-Stack Architecture:**
    *   **Frontend:** React/Docusaurus with a custom chat widget.
    *   **Backend:** FastAPI (Python) handling authentication, conversation history, and LLM orchestration.
    *   **Vector Database:** Qdrant Cloud for storing and retrieving document embeddings.
    *   **Database:** PostgreSQL (via Neon) for user management and chat history.
    *   **AI:** OpenAI-compatible Gemini endpoints (`gemini-embedding-2` for embeddings, `llama-3.1-8b-instant` via Groq for fast inference).

## 📚 Book Contents

The book takes a hands-on, project-based approach covering:
- **Module 1:** ROS 2 Architecture (Nodes, Topics, Services, Actions)
- **Module 2:** Simulation Environments (URDF, Gazebo, Unity)
- **Module 3:** Advanced Perception & Navigation (Isaac ROS, Isaac Sim, Nav2 Biped)
- **Module 4:** The Robot Brain (Whisper, LLM Planning, Vision-Language-Action Pipelines)
- **Capstone:** Building an end-to-end autonomous humanoid assistant.

---

## 🚀 Getting Started (Local Development)

To run the full stack locally for development or reading, follow these steps.

### Prerequisites
*   Node.js (v18+)
*   Python (3.12+)
*   A [Qdrant Cloud](https://cloud.qdrant.io/) free cluster
*   A PostgreSQL database (e.g., [Neon](https://neon.tech/))
*   API keys for [Google Gemini](https://aistudio.google.com/) and [Groq](https://console.groq.com/)

### 1. Clone the Repository
```bash
git clone https://github.com/yourusername/humanoid-robotics-book.git
cd humanoid-robotics-book
```

### 2. Backend Setup
Navigate to the backend directory and set up your Python environment:
```bash
cd backend
python -m venv .venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate
pip install -r requirements.txt
```

Create a `.env` file in the `backend` directory with your credentials:
```ini
DATABASE_URL='postgresql://<user>:<password>@<host>/<db>?sslmode=require'
QDRANT_URL='https://<your-cluster-url>.qdrant.io'
QDRANT_API_KEY='your-qdrant-api-key'
GEMINI_API_KEY='your-gemini-api-key'
GROQ_API_KEY='your-groq-api-key'
SECRET_KEY='your-random-jwt-secret'
```

**Initialize the Database & Embeddings:**
```bash
# Run database migrations
alembic upgrade head

# Generate embeddings from the docs and populate Qdrant
python scripts/embed.py
```

**Start the Backend Server:**
```bash
export PYTHONPATH="src" # On Windows: $env:PYTHONPATH="src"
uvicorn src.main:app --host 127.0.0.1 --port 8000 --reload
```

### 3. Frontend Setup
Open a new terminal window, navigate to the root directory, and start the Docusaurus frontend:
```bash
npm install
npm start
```
The website will be available at `http://localhost:3000`.

---

## 🛠️ Architecture Details

### RAG Pipeline
When a user asks a question via the chat widget:
1.  The query is sent to the FastAPI backend.
2.  The backend uses `gemini-embedding-2` to convert the query into a 3072-dimensional vector.
3.  The vector is used to perform a cosine similarity search against the Qdrant database.
4.  The top 3 most relevant textbook chunks are retrieved.
5.  A prompt is constructed containing the system instructions, the retrieved context, and the user's question.
6.  The prompt is sent to `llama-3.1-8b-instant` (via Groq) to generate a fast, accurate, and context-bound response.

### Security
*   **JWT Authentication:** The chat feature requires users to create an account. Passwords are securely hashed using `bcrypt`, and session management is handled via JWTs.
*   **Strict CORS:** The backend is configured to only accept requests from trusted frontend origins.

## 🤝 Contributing
Contributions are welcome! Whether it's fixing a typo in the documentation, adding a new chapter, or improving the RAG backend, please feel free to submit a Pull Request.

## 📄 License
This project is open-source. Please see the `LICENSE` file for more details.
