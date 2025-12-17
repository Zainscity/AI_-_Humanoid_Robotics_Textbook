from fastapi import APIRouter, Depends
from sqlalchemy.orm import Session
from src.core.database import get_db
from src.services import history_service
from src.core.security import get_current_user
from src.models.schemas import User

router = APIRouter()

@router.get("/conversations")
def get_conversations(db: Session = Depends(get_db), current_user: User = Depends(get_current_user)):
    return history_service.get_conversations(db, user_id=current_user.id)

@router.delete("/conversations/{conversation_id}")
def delete_conversation(conversation_id: str, db: Session = Depends(get_db), current_user: User = Depends(get_current_user)):
    return history_service.delete_conversation(db, conversation_id=conversation_id, user_id=current_user.id)
