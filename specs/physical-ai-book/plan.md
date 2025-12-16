# Architectural Plan: Physical AI & Humanoid Robotics Project

This document outlines the architectural decisions and implementation phases for the project defined in `spec.md`.

## 1. Overall Architecture

The project consists of two primary components:
1.  **Frontend:** A static Docusaurus website containing the book's content.
2.  **Backend:** A FastAPI application serving the RAG (Retrieval-Augmented Generation) chatbot.

These components will be decoupled. The Docusaurus frontend will be deployed to GitHub Pages, while the FastAPI backend will be designed for deployment on a separate hosting service (e.g., Railway, Heroku).

## 2. Phase 1: Docusaurus Foundation & Content Structure

This phase focuses on preparing the Docusaurus environment and creating the book's structure.

- **Decision:** Repurpose the existing Docusaurus site in the `humanoid_robotics` directory to accelerate setup.
- **Steps:**
    1.  **Reconfigure:** Update `docusaurus.config.js` with the new book's metadata (title, URL, project name).
    2.  **Cleanup:** Remove all placeholder content from the `docs` and `blog` directories.
    3.  **Structure Creation:**
        - Create a directory for each of the four modules inside `humanoid_robotics/docs`.
        - Add a `_category_.json` to each directory to define its title and position in the sidebar.
        - Create an `intro.md` file as the main entry point for the documentation.
    4.  **Navigation:** Update `sidebars.js` to correctly order and display the new content structure.
    5.  **Homepage:** Update `src/pages/index.js` to reflect the book's theme and purpose.
    6.  **Content Population:** Iteratively generate the Markdown content for each page of the book based on the outline.

## 3. Phase 2: Backend RAG Service (FastAPI)

This phase focuses on building the AI brain of the chatbot.

- **Directory Structure:** A new top-level `backend` directory will be created to house the FastAPI application, keeping it separate from the frontend.
- **Key Decisions & Rationale:**
    - **Language/Framework:** Python with FastAPI is chosen for its high performance, asynchronous support, excellent data validation (Pydantic), and strong ecosystem for AI/ML tasks.
    - **Vector Database:** Qdrant Cloud will be used as specified for its efficiency in vector similarity search.
    - **Data Ingestion:** A standalone script (`ingest.py`) will be created. It will be responsible for reading the Markdown files from `humanoid_robotics/docs`, chunking the text, generating embeddings via the OpenAI API, and upserting the vectors into Qdrant. This script can be run manually whenever the book content is updated.
- **Steps:**
    1.  Initialize the `backend` directory with a Python virtual environment and a `requirements.txt` file.
    2.  Implement the `ingest.py` script.
    3.  Create the main FastAPI application (`main.py`).
    4.  Implement a `/chat` API endpoint that receives a user's query.
    5.  The endpoint's logic will query Qdrant for relevant context, formulate a prompt for the OpenAI chat completion API, and stream the response back to the frontend.
    6.  Enable CORS (Cross-Origin Resource Sharing) middleware to allow requests from the deployed Docusaurus site.
    7.  All secrets (API keys) will be managed via a `.env` file.

## 4. Phase 3: Frontend Chatbot Integration

This phase focuses on building the user-facing chat interface and integrating it into the Docusaurus site.

- **Key Decisions & Rationale:**
    - **UI Component:** A React-based chat component will be used. This ensures seamless integration with Docusaurus, which is built on React.
    - **Global Integration:** The component will be integrated at a top level in the Docusaurus application, likely by "swizzling" a core theme component, to make it available on all pages.
    - **"Select-to-Ask" Feature:** This will be implemented with client-side JavaScript. A script will listen for text selection events. Upon selection, it will render a small, non-intrusive button that, when clicked, will open the chatbot with the selected text as context.
- **Steps:**
    1.  Create or install a React chat widget component.
    2.  Integrate this component into the main Docusaurus layout.
    3.  Implement the logic to handle chat state (e.g., open/closed, messages).
    4.  Implement the API call from the chat widget to the FastAPI backend.
    5.  Implement the "select-to-ask" JavaScript logic.

## 5. Phase 4: Deployment

- **Frontend (Docusaurus):**
    - Configure and enable the standard GitHub Actions workflow for building the Docusaurus site and deploying it to the `gh-pages` branch of the repository.
- **Backend (FastAPI):**
    - The backend is not a static site and cannot be hosted on GitHub Pages.
    - The plan is to prepare it for deployment on a PaaS (Platform as a Service) provider. This includes creating a `Procfile` for services like Heroku or providing instructions for similar platforms.
    - **Note:** The user will be responsible for creating an account on a hosting provider and deploying the backend service.
