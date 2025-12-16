# Task Breakdown: Physical AI & Humanoid Robotics Project

This document lists the specific, actionable tasks required to implement the project as defined in `plan.md`.

---

### Phase 1: Docusaurus Foundation & Content Structure

- [x] **Task 1.1:** Reconfigure `docusaurus.config.js` with new project metadata. *(Already completed)*
- [x] **Task 1.2:** Clean placeholder content from `blog` and `docs` directories. *(Already completed)*
- [x] **Task 1.3:** Create a main `intro.md` file in the `humanoid_robotics/docs/` directory to serve as the book's landing page.
- [x] **Task 1.4:** Create the directory structure for the four modules within `humanoid_robotics/docs/`:
    - `module-1-ros`
    - `module-2-digital-twin`
    - `module-3-nvidia-isaac`
    - `module-4-vla`
- [x] **Task 1.5:** Add a `_category_.json` file to each module directory to configure its title and position in the sidebar.
- [x] **Task 1.6:** Update the `humanoid_robotics/sidebars.js` file to correctly structure the book's navigation based on the new modules.
- [x] **Task 1.7:** Update the Docusaurus homepage at `humanoid_robotics/src/pages/index.js` to reflect the book's topic and branding.
- [x] **Task 1.8:** Generate a placeholder `index.md` file inside each of the four module directories.
- [x] **Task 1.9:** Populate modules with several submodule placeholder pages to create a deeper content structure.

---

### Phase 2: Backend RAG Service (FastAPI)

- [x] **Task 2.1:** Create the root `backend/` directory for the FastAPI application.
- [x] **Task 2.2:** Create a `backend/requirements.txt` file listing all necessary Python dependencies (`fastapi`, `uvicorn`, `python-dotenv`, `qdrant-client`, `openai`, `langchain`, `pypdf`, `markdown`).
- [x] **Task 2.3:** Create a `backend/.env.example` file to serve as a template for the required environment variables (API keys for OpenAI and Qdrant).
- [x] **Task 2.4:** Implement the basic FastAPI application skeleton in `backend/main.py`, including CORS middleware to allow frontend access.
- [x] **Task 2.5:** Create the data ingestion script `backend/ingest.py`. This script will be responsible for:
    - Reading all `.md` files from the `humanoid_robotics/docs/` directory.
    - Chunking the text content.
    - Generating embeddings using the OpenAI API.
    - Storing the resulting vectors and metadata in a Qdrant Cloud collection.
- [x] **Task 2.6:** Implement the `/chat` API endpoint in `backend/main.py`. This endpoint will:
    - Accept a user query.
    - Generate an embedding for the query.
    - Perform a similarity search against the Qdrant collection.
    - Construct a detailed prompt including the retrieved context and the user query.
    - Send the prompt to the OpenAI chat completion API and stream the response.

---

### Phase 3: Frontend Chatbot Integration

- [x] **Task 3.1:** Create a new React component `ChatWidget` in the `humanoid_robotics/src/components/` directory.
- [x] **Task 3.2:** Design the UI for the `ChatWidget`, including a message display area, a text input field, a send button, and a floating button to toggle the widget's visibility.
- [x] **Task 3.3:** Use Docusaurus's "swizzling" feature to wrap the root layout of the site, allowing the `ChatWidget` to be rendered globally on all pages.
- [x] **Task 3.4:** Implement the client-side logic within the `ChatWidget` to manage its state (e.g., visibility, message history, loading status).
- [x] **Task 3.5:** Implement the API call from the `ChatWidget` to the backend's `/chat` endpoint and display the streamed response.
- [x] **Task 3.6:** Implement the "select-to-ask" functionality. This will involve a script that listens for text selection events and presents a button to send the selected text to the chatbot as context.

---

### Phase 4: Deployment

- [x] **Task 4.1:** Create a GitHub Actions workflow file (`.github/workflows/deploy.yml`) to automate the build and deployment of the Docusaurus site to GitHub Pages.
- [x] **Task 4.2:** Create a `Procfile` and `runtime.txt` in the `backend/` directory to facilitate deployment on common PaaS platforms like Heroku or Railway.
- [x] **Task 4.3:** Create a `DEPLOYMENT.md` file with clear, step-by-step instructions for deploying the backend service and configuring the necessary environment variables.
