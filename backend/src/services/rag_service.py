from qdrant_client import QdrantClient, models
from openai import OpenAI
from src.core.config import settings
from src.models import conversation, message, user
import uuid
from . import history_service

qdrant_client = QdrantClient(
    url=settings.QDRANT_URL,
    api_key=settings.QDRANT_API_KEY
)

embedding_client = OpenAI(
    api_key=settings.GEMINI_API_KEY,
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
)

external_client = OpenAI(
    api_key=settings.GROQ_API_KEY,
    base_url="https://api.groq.com/openai/v1"
)

def get_chat_completion(messages):
    return external_client.chat.completions.create(
        model="llama-3.1-8b-instant",
        messages=messages
    )

def save_message(db, conversation_id, content, is_from_user):
    new_message = message.Message(
        id=uuid.uuid4(),
        conversation_id=conversation_id,
        content=content,
        is_from_user=is_from_user
    )
    db.add(new_message)
    db.commit()
    db.refresh(new_message)
    return new_message

def selected_query(query: str, context: str, conversation_id: str, db, current_user: user.User):
    if conversation_id is None:
        conversation_id = history_service.create_conversation(db, current_user.id).id

    messages = [
        {"role": "system", "content": "You are the official AI RAG Chatbot for the Humanoid Robotics book. Your answers must come ONLY from: Qdrant retrieved chunks, or User-selected text provided in the query. If the answer is not found in these sources, respond with: 'The answer is not found in the book.'"},
        {"role": "user", "content": f"Context: {context}\n\nQuestion: {query}"}
    ]
    response = get_chat_completion(messages)
    
    save_message(db, conversation_id, query, True)
    bot_response = response.choices[0].message.content
    save_message(db, conversation_id, bot_response, False)

    return {"message": bot_response, "conversation_id": conversation_id}

def query(query: str, conversation_id: str, db, current_user: user.User):
    response = embedding_client.embeddings.create(
        input=query,
        model="models/text-embedding-004"
    )
    query_embedding = response.data[0].embedding

    search_result = qdrant_client.search(
        collection_name="humanoid_robotics_book",
        query_vector=query_embedding,
        limit=3
    )

    context = "\n".join([hit.payload["text"] for hit in search_result])
    
    if conversation_id is None:
        conversation_id = history_service.create_conversation(db, current_user.id).id

    messages = [
        {"role": "system", "content": "You are the official AI RAG Chatbot for the Humanoid Robotics book. Your answers must come ONLY from: Qdrant retrieved chunks, or User-selected text provided in the query. If the answer is not found in these sources, respond with: 'The answer is not found in the book.'"},
        {"role": "user", "content": f"Context: {context}\n\nQuestion: {query}"}
    ]
    response = get_chat_completion(messages)
    
    save_message(db, conversation_id, query, True)
    bot_response = response.choices[0].message.content
    save_message(db, conversation_id, bot_response, False)

    return {"message": bot_response, "conversation_id": conversation_id}
