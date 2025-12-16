import os
import argparse
from dotenv import load_dotenv
from langchain_core.documents import Document
from langchain_text_splitters import MarkdownTextSplitter
from langchain_community.embeddings import HuggingFaceEmbeddings
from langchain_qdrant import QdrantVectorStore
from qdrant_client import QdrantClient, models
import mimetypes

# Ensure environment variables are loaded
load_dotenv()

# --- Configuration ---
DOCS_DIR = os.path.join(os.getcwd(), 'humanoid_robotics', 'docs')
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
COLLECTION_NAME = "physical_ai_book_rag"

# --- Constants for Langchain ---
CHUNK_SIZE = 1000
CHUNK_OVERLAP = 100

def get_markdown_files(directory):
    """
    Recursively finds all markdown files (.md, .mdx) in the given directory.
    """
    markdown_files = []
    for root, _, files in os.walk(directory):
        for file in files:
            # Check for both .md and .mdx extensions
            if file.endswith((".md", ".mdx")):
                markdown_files.append(os.path.join(root, file))
    return markdown_files

def extract_text_from_markdown(file_path):
    """
    Extracts text content from a markdown file.
    This can be enhanced to parse frontmatter, headers, etc.
    """
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()
    
    # Simple text extraction. Langchain's splitter will handle structure.
    return content

import re

def clean_docusaurus_path(path: str) -> str:
    """
    Cleans a file path to match Docusaurus's URL slugification rules.
    - Removes .md or .mdx extension.
    - Removes numeric prefixes (e.g., '1-', '01-') from the filename part.
    """
    # Split path into directory and filename
    dirname, basename = os.path.split(path)

    # Remove extension
    name_without_ext, _ = os.path.splitext(basename)

    # Remove numeric prefix from filename (e.g., '1-intro' -> 'intro')
    cleaned_name = re.sub(r'^\d+-', '', name_without_ext)

    # Reconstruct path
    return os.path.join(dirname, cleaned_name).replace('\\', '/')

def ingest_data(clear_existing=False):
    """
    Main function to ingest markdown data into Qdrant.
    """
    if not QDRANT_URL or not QDRANT_API_KEY:
        print("Error: QDRANT_URL or QDRANT_API_KEY not set in environment variables.")
        print("Please create a .env file in the backend directory with these variables.")
        return

    print(f"Starting data ingestion from {DOCS_DIR}...")

    # --- Step 1: Initialize HuggingFace Embeddings (runs locally) ---
    print("Initializing local embedding model (all-MiniLM-L6-v2)...")
    embeddings = HuggingFaceEmbeddings(model_name="all-MiniLM-L6-v2")

    # --- Step 2: Initialize Qdrant Client ---
    qdrant_client = QdrantClient(
        url=QDRANT_URL,
        api_key=QDRANT_API_KEY,
    )

    # --- Step 3: Clear existing collection if requested ---
    if clear_existing:
        print(f"Attempting to delete existing collection '{COLLECTION_NAME}'...")
        try:
            qdrant_client.delete_collection(collection_name=COLLECTION_NAME)
            print(f"Collection '{COLLECTION_NAME}' deleted.")
        except Exception as e:
            print(f"Could not delete collection (it might not exist): {e}")

    # --- Step 4: Get and Process Markdown Files ---
    markdown_files = get_markdown_files(DOCS_DIR)
    if not markdown_files:
        print(f"No markdown files found in {DOCS_DIR}. Exiting.")
        return

    print(f"Found {len(markdown_files)} markdown documents. Processing...")

    all_docs = []
    text_splitter = MarkdownTextSplitter(chunk_size=CHUNK_SIZE, chunk_overlap=CHUNK_OVERLAP)

    for file_path in markdown_files:
        content = extract_text_from_markdown(file_path)
        relative_path = os.path.relpath(file_path, DOCS_DIR)
        
        # Split markdown content into chunks
        chunks = text_splitter.split_text(content)
        
        for i, chunk in enumerate(chunks):
            # Basic metadata, can be enhanced with title extraction from frontmatter
            metadata = {
                "source": clean_docusaurus_path(relative_path),
                "chunk_id": i,
            }
            doc = Document(page_content=chunk, metadata=metadata)
            all_docs.append(doc)

    print(f"Generated {len(all_docs)} text chunks.")

    # --- Step 5: Upload to Qdrant ---
    if all_docs:
        print(f"Uploading chunks to Qdrant collection '{COLLECTION_NAME}'...")
        try:
            QdrantVectorStore.from_documents(
                documents=all_docs,
                embedding=embeddings,
                collection_name=COLLECTION_NAME,
                url=QDRANT_URL,
                api_key=QDRANT_API_KEY,
                force_recreate=True, # Recreate collection to ensure it's clean
                content_payload_key="page_content",
                metadata_payload_key="metadata",
            )
            print(f"Successfully uploaded {len(all_docs)} chunks to Qdrant.")
        except Exception as e:
            print(f"Error uploading to Qdrant: {e}")
            print("Please ensure your Qdrant instance is running and accessible, and API key is correct.")
    else:
        print("No chunks to upload to Qdrant.")

    print("Data ingestion process completed.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Ingest markdown documents into Qdrant for RAG.")
    parser.add_argument("--clear", action="store_true", help="Clear existing Qdrant collection before ingesting.")
    args = parser.parse_args()

    ingest_data(clear_existing=args.clear)