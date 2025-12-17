from fastapi import APIRouter, Depends, HTTPException, Body
from sqlalchemy.orm import Session
from src.core.database import get_db
from src.services import rag_service

router = APIRouter()

@router.post("/")
def query(query: str = Body(..., embed=True), db: Session = Depends(get_db)):
    return rag_service.query(query, db)

@router.post("/selected-query")
def selected_query(query: str = Body(..., embed=True), context: str = Body(..., embed=True), db: Session = Depends(get_db)):
    return rag_service.selected_query(query, context, db)
