
import os
import google.generativeai as genai
from dotenv import load_dotenv

def verify_embedding_quota():
    """
    Attempts a single embedding request to the Gemini API to verify quota.
    """
    print("--- Starting Gemini API Quota Verification ---")
    
    # 1. Load environment variables from .env file
    print("Loading environment variables from backend/.env...")
    dotenv_path = os.path.join(os.path.dirname(__file__), '.env')
    load_dotenv(dotenv_path=dotenv_path)
    
    api_key = os.getenv("GEMINI_API_KEY")
    
    if not api_key:
        print("\n[ERROR] GEMINI_API_KEY not found in backend/.env file.")
        print("Please ensure the .env file exists in the 'backend' directory and contains your key.")
        print("--- Verification Failed ---")
        return

    print("GEMINI_API_KEY found.")

    # 2. Configure the Gemini API client
    try:
        genai.configure(api_key=api_key)
        print("Gemini API client configured successfully.")
    except Exception as e:
        print(f"\n[ERROR] Failed to configure Gemini API client: {e}")
        print("--- Verification Failed ---")
        return

    # 3. Attempt to make a single embedding request
    try:
        print("Attempting to embed content with 'models/embedding-001'...")
        result = genai.embed_content(
            model="models/embedding-001",
            content="This is a test to verify API quota.",
            task_type="RETRIEVAL_QUERY"
        )
        
        # 4. Check the result
        if result and 'embedding' in result:
            print("\n[SUCCESS] Successfully received embedding from the API.")
            print(f"Embedding vector length: {len(result['embedding'])}")
            print("This confirms your API key and project quota are working correctly for embeddings.")
            print("--- Verification Succeeded ---")
        else:
            print(f"\n[ERROR] API call succeeded but returned an unexpected result: {result}")
            print("--- Verification Failed ---")

    except Exception as e:
        print("\n[ERROR] The API call failed. See the error details below:")
        print("---------------------------------------------------------")
        print(e)
        print("---------------------------------------------------------")
        print("\nThis error almost certainly confirms the quota issue in your Google Cloud project.")
        print("Please check your project's billing and quotas in the Google Cloud Console.")
        print("--- Verification Failed ---")

if __name__ == "__main__":
    verify_embedding_quota()
