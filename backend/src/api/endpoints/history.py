from fastapi import APIRouter, Depends
from sqlalchemy.orm import Session
from src.core.database import get_db
from src.services import history_service
from src.core.security import get_current_user
from src.models.user import User

router = APIRouter()

@router.get("/conversations")
def get_conversations(db: Session = Depends(get_db), current_user: User = Depends(get_current_user)):
    return history_service.get_conversations(db, current_user.id)

@router.post("/conversations")
def create_conversation(db: Session = Depends(get_db), current_user: User = Depends(get_current_user)):
    return history_service.create_conversation(db, current_user.id)

@router.get("/conversations/{conversation_id}/messages")
def get_conversation_messages(conversation_id: str, db: Session = Depends(get_db), current_user: User = Depends(get_current_user)):
    return history_service.get_messages(db, conversation_id, current_user.id)

@router.delete("/conversations/{conversation_id}")
def delete_conversation(conversation_id: str, db: Session = Depends(get_db), current_user: User = Depends(get_current_user)):
    return history_service.delete_conversation(db, conversation_id, current_user.id)
