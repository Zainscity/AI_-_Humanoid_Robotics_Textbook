import uuid
from sqlalchemy import Column, DateTime, ForeignKey
from sqlalchemy.dialects.postgresql import UUID, JSONB
from sqlalchemy.sql import func
from src.core.database import Base

class RetrievalLog(Base):
    __tablename__ = "retrieval_logs"
    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    message_id = Column(UUID(as_uuid=True), ForeignKey("messages.id"), nullable=False)
    retrieved_context = Column(JSONB, nullable=False)
    created_at = Column(DateTime(timezone=True), server_default=func.now())
