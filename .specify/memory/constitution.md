# Project Constitution: Physical AI & Humanoid Robotics

This document outlines the guiding principles, standards, and methodologies for the "Physical AI & Humanoid Robotics" book and its integrated RAG chatbot.

## 1. Core Principles

- **Goal-Oriented:** The primary objective is to bridge the gap between digital AI and physical embodiment, providing students with a practical, hands-on guide to controlling humanoid robots.
- **Clarity and Practicality:** All content, both text and code, must be clear, concise, and aimed at a technical student audience. Concepts will be followed by immediate, practical application.
- **Reproducibility:** Code examples and project setups must be fully reproducible. All dependencies and environment configurations will be explicitly documented.
- **Spec-Driven Development:** We will follow a structured approach: define the specifications (plan), generate the content (book), implement the features (code), and integrate the AI (chatbot).

## 2. Content & Code Standards

- **Tone:** The book's voice will be authoritative, educational, and engaging.
- **Structure:** Each module will begin with a theoretical overview, followed by step-by-step tutorials.
- **Code Quality (Python & ROS):**
    - All Python code must adhere to the PEP 8 style guide.
    - It must be fully type-hinted.
    - Code must be accompanied by comments that explain the *why* behind the implementation, not just the *what*.
    - All code snippets must be complete and runnable within the context of their respective tutorials.
- **Code Quality (Docusaurus & Frontend):**
    - Frontend components will be written in React.
    - Code will follow modern JavaScript/TypeScript best practices.

## 3. Technical & Architectural Guidelines

- **Single Source of Truth:** The Markdown files of the Docusaurus book are the single source of truth for the RAG chatbot's knowledge base.
- **Backend (FastAPI):**
    - The API will be robust, with clear data contracts (Pydantic models).
    - It will be responsible for the entire RAG pipeline: receiving queries, searching the vector DB, querying the LLM, and returning answers.
- **Chatbot UX:**
    - The chat interface must be intuitive and non-intrusive to the reading experience.
    - The "select text to ask" feature is a primary requirement and must be implemented seamlessly.
- **Credentials Management:** All API keys and secrets (OpenAI, Qdrant, Neon) **must not** be hard-coded. They will be managed via a `.env` file in the backend directory.
- **Dependency Management:**
    - Node.js dependencies will be managed via `package.json`.
    - Python dependencies will be managed via a `requirements.txt` file within the `backend` directory.

## 4. RAG Chatbot Principles

- **Context-Bound:** The chatbot must primarily derive its answers from the book content retrieved from the Qdrant vector store.
- **Honesty:** If a confident answer cannot be found within the provided context, the chatbot should state that the information is not available in the book rather than speculating.

## 5. Versioning & Deployment

- **Version Control:** All changes will be managed using Git.
- **Deployment:** The Docusaurus site will be configured for automated deployment to GitHub Pages.