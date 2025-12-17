from qdrant_client import QdrantClient, models
from openai import OpenAI
from src.core.config import settings
from src.models import conversation, message, user
import uuid

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

def selected_query(query: str, context: str, conversation_id: str, db, current_user: user.User):
    if conversation_id is None:
        new_conversation = conversation.Conversation(id=uuid.uuid4(), user_id=current_user.id)
        db.add(new_conversation)
        db.commit()
        db.refresh(new_conversation)
        conversation_id = new_conversation.id

    response = external_client.chat.completions.create(
        model="llama-3.1-8b-instant",
        messages=[
            {"role": "system", "content": "You are the official AI RAG Chatbot for the Humanoid Robotics book. Your answers must come ONLY from: Qdrant retrieved chunks, or User-selected text provided in the query. If the answer is not found in these sources, respond with: 'The answer is not found in the book.'"},
            {"role": "user", "content": f"Context: {context}\n\nQuestion: {query}"}
        ]
    )

    user_message = message.Message(id=uuid.uuid4(), conversation_id=conversation_id, content=query, is_from_user=True)
    bot_message = message.Message(id=uuid.uuid4(), conversation_id=conversation_id, content=response.choices[0].message.content, is_from_user=False)
    db.add(user_message)
    db.add(bot_message)
    db.commit()

    return {"message": response.choices[0].message.content, "conversation_id": conversation_id}

def query(query: str, conversation_id: str, db, current_user: user.User):
    # Create embedding for the query
    response = embedding_client.embeddings.create(
        input=query,
        model="models/text-embedding-004"
    )
    query_embedding = response.data[0].embedding

    # Search Qdrant for similar document chunks
    search_result = qdrant_client.search(
        collection_name="humanoid_robotics_book",
        query_vector=query_embedding,
        limit=3 # Retrieve top 3 relevant chunks
    )

    # Extract context from retrieved chunks
    context = "\n".join([hit.payload["text"] for hit in search_result])
    
    if conversation_id is None:
        new_conversation = conversation.Conversation(id=uuid.uuid4(), user_id=current_user.id)
        db.add(new_conversation)
        db.commit()
        db.refresh(new_conversation)
        conversation_id = new_conversation.id


    response = external_client.chat.completions.create(
        model="llama-3.1-8b-instant",
        messages=[
            {"role": "system", "content": "You are the official AI RAG Chatbot for the Humanoid Robotics book. Your answers must come ONLY from: Qdrant retrieved chunks, or User-selected text provided in the query. If the answer is not found in these sources, respond with: 'The answer is not found in the book.'"},
            {"role": "user", "content": f"Context: {context}\n\nQuestion: {query}"}
        ]
    )
    
    user_message = message.Message(id=uuid.uuid4(), conversation_id=conversation_id, content=query, is_from_user=True)
    bot_message = message.Message(id=uuid.uuid4(), conversation_id=conversation_id, content=response.choices[0].message.content, is_from_user=False)
    db.add(user_message)
    db.add(bot_message)
    db.commit()

    return {"message": response.choices[0].message.content, "conversation_id": conversation_id}
