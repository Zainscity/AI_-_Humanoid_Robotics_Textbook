from unittest.mock import patch
from src.services import rag_service

@patch("src.services.rag_service.QdrantClient")
@patch("src.services.rag_service.OpenAI")
def test_query(mock_openai, mock_qdrant):
    # This is a placeholder test. A real test would require more setup and mocking.
    assert True
