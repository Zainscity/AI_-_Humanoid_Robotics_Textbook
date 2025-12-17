from sqlalchemy.orm import Session
from src.models import conversation, message
import uuid

def get_conversations(db: Session, user_id: str):
    return db.query(conversation.Conversation).filter(conversation.Conversation.user_id == user_id).all()

def get_messages(db: Session, conversation_id: str, user_id: str):
    # Ensure the user owns the conversation before returning messages
    conv = db.query(conversation.Conversation).filter(conversation.Conversation.id == conversation_id, conversation.Conversation.user_id == user_id).first()
    if not conv:
        return []
    return db.query(message.Message).filter(message.Message.conversation_id == conversation_id).all()

def delete_conversation(db: Session, conversation_id: str, user_id: str):
    conv = db.query(conversation.Conversation).filter(conversation.Conversation.id == conversation_id, conversation.Conversation.user_id == user_id).first()
    if conv:
        db.query(message.Message).filter(message.Message.conversation_id == conversation_id).delete()
        db.delete(conv)
        db.commit()
        return {"message": "Conversation deleted successfully"}
    return {"message": "Conversation not found or not authorized"}

def create_conversation(db: Session, user_id: str):
    new_conversation = conversation.Conversation(id=uuid.uuid4(), user_id=user_id)
    db.add(new_conversation)
    db.commit()
    db.refresh(new_conversation)
    return new_conversation