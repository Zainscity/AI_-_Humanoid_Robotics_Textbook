# Data Model: Integrated RAG Chatbot

This document defines the data model for the Integrated RAG Chatbot feature.

## Entities

### User

Represents a user of the chatbot.

-   `id` (uuid, primary key)
-   `email` (string, unique)
-   `hashed_password` (string)
-   `created_at` (timestamp)

### Conversation

Represents a single conversation between a user and the chatbot.

-   `id` (uuid, primary key)
-   `user_id` (uuid, foreign key to User)
-   `created_at` (timestamp)

### Message

Represents a single message in a conversation.

-   `id` (uuid, primary key)
-   `conversation_id` (uuid, foreign key to Conversation)
-   `content` (text)
-   `is_from_user` (boolean)
-   `created_at` (timestamp)

### RetrievalLog

Records the context retrieved from the vector database for a given query.

-   `id` (uuid, primary key)
-   `message_id` (uuid, foreign key to Message)
-   `retrieved_context` (jsonb)
-   `created_at` (timestamp)

## Relationships

-   A `User` can have many `Conversation`s.
-   A `Conversation` belongs to one `User` and has many `Message`s.
-   A `Message` belongs to one `Conversation` and can have one `RetrievalLog`.
-   A `RetrievalLog` belongs to one `Message`.
