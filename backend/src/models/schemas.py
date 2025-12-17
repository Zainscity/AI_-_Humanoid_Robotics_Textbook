from pydantic import BaseModel
import uuid
from datetime import datetime

class UserBase(BaseModel):
    email: str

class UserCreate(UserBase):
    password: str

class User(UserBase):
    id: uuid.UUID

    class Config:
        orm_mode = True

class ConversationBase(BaseModel):
    pass

class ConversationCreate(ConversationBase):
    pass

class Conversation(ConversationBase):
    id: uuid.UUID
    user_id: uuid.UUID
    created_at: datetime

    class Config:
        orm_mode = True

class MessageBase(BaseModel):
    content: str
    is_from_user: bool

class MessageCreate(MessageBase):
    pass

class Message(MessageBase):
    id: uuid.UUID
    conversation_id: uuid.UUID
    created_at: datetime

    class Config:
        orm_mode = True

class RetrievalLogBase(BaseModel):
    retrieved_context: dict

class RetrievalLogCreate(RetrievalLogBase):
    pass

class RetrievalLog(RetrievalLogBase):
    id: uuid.UUID
    message_id: uuid.UUID
    created_at: datetime

    class Config:
        orm_mode = True
