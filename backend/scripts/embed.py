import os
import sys
import tiktoken
from langchain.text_splitter import RecursiveCharacterTextSplitter
from qdrant_client import QdrantClient, models
from openai import OpenAI
import httpx
import uuid

# Add the project root to the Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.core.config import settings


def get_mdx_files(path):
    all_files = []
    for root, _, files in os.walk(path):
        for file in files:
            if file.endswith(".md") or file.endswith(".mdx"):
                all_files.append(os.path.join(root, file))
    return all_files

def get_chunks(text):
    text_splitter = RecursiveCharacterTextSplitter(
        chunk_size=500,
        chunk_overlap=50,
        length_function=len
    )
    chunks = text_splitter.split_text(text)
    return chunks

def main():
    qdrant_client = QdrantClient(
        url=settings.QDRANT_URL,
        api_key=settings.QDRANT_API_KEY
    )

    # Explicitly create an httpx client with proxies disabled
    http_client = httpx.Client(proxies=None)

    embedding_client = OpenAI(
        api_key=settings.GEMINI_API_KEY,
        base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
        http_client=http_client
    )

    qdrant_client.recreate_collection(
        collection_name="humanoid_robotics_book",
        vectors_config=models.VectorParams(size=768, distance=models.Distance.COSINE),
    )

    docs_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "docs")
    mdx_files = get_mdx_files(docs_path)

    for file_path in mdx_files:
        try:
            with open(file_path, "r", encoding="utf-8") as f:
                content = f.read()
                chunks = get_chunks(content)
                for i, chunk in enumerate(chunks):
                    response = embedding_client.embeddings.create(
                        input=chunk,
                        model="models/text-embedding-004"
                    )
                    point_id = str(uuid.uuid5(uuid.NAMESPACE_URL, f"{file_path}-{i}"))
                    qdrant_client.upsert(
                        collection_name="humanoid_robotics_book",
                        points=[
                            models.PointStruct(
                                id=point_id,
                                vector=response.data[0].embedding,
                                payload={"file_path": file_path, "chunk_index": i, "text": chunk}
                            )
                        ],
                        wait=True
                    )
        except Exception as e:
            print(f"Error processing file {file_path}: {e}")


if __name__ == "__main__":
    main()