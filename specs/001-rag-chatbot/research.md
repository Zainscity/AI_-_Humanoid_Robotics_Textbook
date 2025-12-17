# Research: Integrated RAG Chatbot

## Research Task 1: Best practices for chunking and embedding MDX content

**Decision**: Use a recursive character text splitter with a chunk size of 500 tokens and an overlap of 50 tokens. This provides a good balance between context preservation and embedding granularity. For embeddings, use OpenAI's `text-embedding-ada-002` model due to its performance and cost-effectiveness.

**Rationale**: A recursive character text splitter is effective for structured content like MDX as it tries to keep paragraphs, sentences, and lines together. The chosen chunk size and overlap are common starting points for RAG pipelines and can be tuned later if needed. The `text-embedding-ada-002` model is a well-established and high-performing choice for semantic search.

**Alternatives considered**:
-   **Fixed-size chunking**: Simpler but can break up sentences and lose context.
-   **Markdown-specific chunking**: More complex to implement and might not be necessary for this use case.
-   **Other embedding models**: `text-embedding-3-small` and `text-embedding-3-large` from OpenAI, or open-source models like those from Sentence-Transformers. `ada-002` is a good starting point for now.

## Research Task 2: Evaluate different React chat widget libraries

**Decision**: Use a custom-built React component for the chat widget.

**Rationale**: While there are many chat widget libraries available, building a custom component will provide the most flexibility for a seamless integration with the Docusaurus theme and for implementing the specific features required, such as the "chat about this" functionality for selected text. It also avoids adding another third-party dependency to the project.

**Alternatives considered**:
-   **react-chat-widget**: A popular and easy-to-use library, but customization options are limited.
-   **Chatwoot**: A more feature-rich open-source solution, but it's a full-fledged customer support platform and would be overkill for this project.

## Research Task 3: Determine the optimal Qdrant configuration

**Decision**: Use the Qdrant Cloud Free Tier with an in-memory storage configuration and a single collection for the book's content.

**Rationale**: The free tier of Qdrant Cloud is sufficient for the initial version of this feature. In-memory storage will provide the best performance for the expected data volume. A single collection is appropriate as all the content comes from a single source (the book).

**Alternatives considered**:
-   **Self-hosting Qdrant**: More complex to set up and maintain.
-   **Using on-disk storage**: Slower than in-memory storage, but more cost-effective for larger datasets.

## Research Task 4: Research best practices for securing a FastAPI backend and managing user authentication

**Decision**: Use JWTs (JSON Web Tokens) for user authentication. The backend will have a `/token` endpoint that returns a JWT upon successful login. The JWT will be stored in an HttpOnly cookie on the client-side. All other endpoints will require a valid JWT.

**Rationale**: JWTs are a standard and secure way to handle user authentication in modern web applications. Storing the JWT in an HttpOnly cookie helps to mitigate XSS attacks.

**Alternatives considered**:
-   **Session-based authentication**: More stateful and less scalable than JWTs.
-   **OAuth2**: More complex to implement and might be overkill for this initial version, but can be added later if needed.
