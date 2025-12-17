from src.core.security import get_password_hash, verify_password, create_access_token

def test_password_hashing():
    password = "testpassword"
    hashed_password = get_password_hash(password)
    assert verify_password(password, hashed_password)

def test_create_access_token():
    data = {"sub": "test@test.com"}
    token = create_access_token(data)
    assert isinstance(token, str)
