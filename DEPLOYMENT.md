# Deployment Instructions

This document provides instructions for deploying the Docusaurus frontend and the FastAPI backend.

## Frontend (Docusaurus)

The frontend is a Docusaurus static website. It is configured to deploy automatically to GitHub Pages whenever you push to the `main` branch.

### Prerequisites

- A GitHub repository for your project.
- The deployment workflow file at `.github/workflows/deploy.yml` (already created).

### Deployment Steps

1.  **Push to `main`**: Simply push your code to the `main` branch of your GitHub repository.
2.  **Enable GitHub Pages**:
    - Go to your repository's **Settings** tab.
    - In the **Pages** section, select the `gh-pages` branch as the source.
    - Save the changes.

Your Docusaurus website will be deployed to `https://<your-username>.github.io/<your-repository-name>/`.

## Backend (FastAPI)

The backend is a FastAPI application that serves the RAG chatbot. It needs to be deployed to a Platform-as-a-Service (PaaS) provider like Heroku or Railway. These instructions will use Heroku as an example.

### Prerequisites

- A free [Heroku account](https://signup.heroku.com/).
- The [Heroku CLI](https://devcenter.heroku.com/articles/heroku-cli) installed on your machine.
- Git installed on your machine.

### Deployment Steps

1.  **Login to Heroku**:
    ```bash
    heroku login
    ```

2.  **Create a new Heroku app**:
    ```bash
    heroku create <your-app-name>
    ```
    (Replace `<your-app-name>` with a unique name for your application).

3.  **Configure Environment Variables**:
    You need to set the following environment variables in your Heroku app. These are the same variables from your `.env` file.

    - `QDRANT_URL`: Your Qdrant Cloud URL.
    - `QDRANT_API_KEY`: Your Qdrant Cloud API key.
    - `GEMINI_API_KEY`: Your Google Gemini API key.
    - `OPENAI_API_KEY`: Your OpenAI API key (if you are using OpenAI).

    You can set these variables using the Heroku CLI:
    ```bash
    heroku config:set QDRANT_URL="<your-qdrant-url>"
    heroku config:set QDRANT_API_KEY="<your-qdrant-api-key>"
    heroku config:set GEMINI_API_KEY="<your-gemini-api-key>"
    ```
    ...and so on for all your required keys.

4.  **Deploy to Heroku**:
    You can deploy the backend by pushing the `backend` subdirectory to your Heroku app's remote. This is a bit tricky, as you only want to push the `backend` folder. The easiest way to do this is to use `git subtree push`.

    ```bash
    git subtree push --prefix backend heroku main
    ```

    This command pushes the contents of the `backend` directory to the `main` branch of your Heroku remote.

5.  **Check the logs**:
    You can check the status of your deployment and see any error messages using the Heroku logs.
    ```bash
    heroku logs --tail
    ```

### Updating the Frontend with the Backend URL

Once your backend is deployed, you will get a URL for your Heroku app (e.g., `https://<your-app-name>.herokuapp.com`). You need to update the `BACKEND_URL` constant in `humanoid_robotics/src/components/ChatWidget/index.js` to point to this new URL.

```javascript
// humanoid_robotics/src/components/ChatWidget/index.js
const BACKEND_URL = "https://<your-app-name>.herokuapp.com"; // Replace with your deployed backend URL
```

After updating this URL, your frontend will be able to communicate with your deployed backend.
