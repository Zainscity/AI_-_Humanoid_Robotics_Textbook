from fastapi import APIRouter, Depends, HTTPException, Body
from sqlalchemy.orm import Session
from src.core.database import get_db
from src.services import rag_service
from typing import Optional
from src.core.security import get_current_user
from src.models.user import User

router = APIRouter()

@router.post("/")
def query(query: str = Body(..., embed=True), conversation_id: Optional[str] = Body(None, embed=True), db: Session = Depends(get_db), current_user: User = Depends(get_current_user)):
    return rag_service.query(query, conversation_id, db, current_user)

@router.post("/selected-query")
def selected_query(query: str = Body(..., embed=True), context: str = Body(..., embed=True), conversation_id: Optional[str] = Body(None, embed=True), db: Session = Depends(get_db), current_user: User = Depends(get_current_user)):
    return rag_service.selected_query(query, context, conversation_id, db, current_user)
