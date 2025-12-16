import os
from typing import List, Optional
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from dotenv import load_dotenv

from langchain_community.embeddings import HuggingFaceEmbeddings
from langchain_openai import ChatOpenAI
from langchain_qdrant import QdrantVectorStore
from qdrant_client import QdrantClient, models

# Ensure environment variables are loaded
load_dotenv()

# --- Configuration ---
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")
COLLECTION_NAME = "physical_ai_book_rag" # Must match collection name in ingest.py
MODEL_NAME = "gpt-3.5-turbo" # Or "gpt-4"

# --- Validate Configuration ---
if not QDRANT_URL or not QDRANT_API_KEY or not OPENAI_API_KEY:
    raise ValueError("QDRANT_URL, QDRANT_API_KEY, or OPENAI_API_KEY for the LLM not set in environment variables.")

# --- Initialize Clients ---
embeddings = HuggingFaceEmbeddings(model_name="all-MiniLM-L6-v2")
llm = ChatOpenAI(model=MODEL_NAME, openai_api_key=OPENAI_API_KEY, streaming=True) # Enable streaming

qdrant_client = QdrantClient(
    url=QDRANT_URL,
    api_key=QDRANT_API_KEY,
)

# Initialize QdrantVectorstore for retrieval
# Ensure this is correctly configured with your collection.
vectorstore = QdrantVectorStore(
    client=qdrant_client,
    collection_name=COLLECTION_NAME,
    embedding=embeddings,
    # This ensures that the vectorstore correctly maps metadata to Langchain's Document format
    content_payload_key="text", # The key in Qdrant payload that holds the document text
    metadata_payload_key="metadata" # The key in Qdrant payload that holds other metadata
)

# Create the FastAPI app
app = FastAPI(
    title="Physical AI & Humanoid Robotics - RAG API",
    description="An API for the RAG chatbot integrated with the Docusaurus book.",
    version="0.1.0",
)

# Set up CORS (Cross-Origin Resource Sharing)
# This allows the Docusaurus frontend to make requests to this backend.
# In a production environment, you should restrict the origins to your actual frontend URL.
origins = [
    "http://localhost:3000",  # Docusaurus dev server
    "https://wanizamunwar.github.io", # Deployed Docusaurus site
    "http://127.0.0.1:3000"
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# --- Pydantic Models ---
class ChatRequest(BaseModel):
    query: str
    selected_text: Optional[str] = None # For select-to-ask feature

# --- Endpoints ---
@app.get("/", tags=["Health Check"])
async def root():
    """
    A simple health check endpoint to confirm the server is running.
    """
    return {"status": "ok", "message": "Welcome to the RAG API!"}

@app.post("/chat", tags=["RAG Chatbot"])
async def chat_with_rag(request: ChatRequest):
    """
    Receives a user query and returns a response from the RAG chatbot.
    Optionally includes selected text from the book for focused answers.
    """
    try:
        # Step 1: Retrieve relevant documents from Qdrant
        # If selected_text is provided, prioritize it for retrieval
        if request.selected_text:
            search_query = request.selected_text + " " + request.query
        else:
            search_query = request.query
            
        # Perform similarity search
        retrieved_docs = vectorstore.similarity_search(search_query, k=4) # Retrieve top 4 chunks

        # Extract content from retrieved documents
        context_texts = [doc.page_content for doc in retrieved_docs]
        context = "\n\n".join(context_texts)

        # Step 2: Construct prompt for LLM
        system_message = (
            "You are a helpful assistant that answers questions about the 'Physical AI & Humanoid Robotics' book."
            "Use only the provided context to answer the question. If you don't know the answer "
            "based on the context, politely state that the information is not available in the book."
            "Your answers should be concise and directly address the user's query."
        )
        
        user_message = f"Context from the book:\n{context}\n\nQuestion: {request.query}"
        if request.selected_text:
            user_message += f"\n\nUser also highlighted: '{request.selected_text}'"

        messages = [
            {"role": "system", "content": system_message},
            {"role": "user", "content": user_message},
        ]

        # Step 3: Get response from LLM (streaming is handled by ChatOpenAI internally)
        # Note: Langchain's streaming with FastAPI requires specific handlers for direct streaming response
        # For true streaming with FastAPI, Server-Sent Events (SSE) or WebSockets would be used.
        response = llm.invoke(messages)
        
        return {"answer": response.content, "source_documents": [doc.metadata for doc in retrieved_docs]}

    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))