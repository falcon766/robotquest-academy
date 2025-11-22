# Infrastructure Setup Guide

Follow these steps to connect your local project to the cloud.

## 1. Git & GitHub (Version Control)
You have already initialized a local Git repository. Now, let's push it to GitHub.

1.  **Create a Repository on GitHub**:
    *   Go to [github.com/new](https://github.com/new).
    *   Name it `roboquest-academy` (or similar).
    *   **Do not** initialize with README, .gitignore, or License (we already have them).
    *   Click **Create repository**.

2.  **Push Local Code**:
    *   Copy the commands under "…or push an existing repository from the command line".
    *   Run them in your terminal (inside the project folder):
        ```bash
        git remote add origin https://github.com/<YOUR_USERNAME>/roboquest-academy.git
        git branch -M main
        git add .
        git commit -m "Initial commit"
        git push -u origin main
        ```

## 2. Firebase (Backend & Auth)
1.  **Create a Project**:
    *   Go to [console.firebase.google.com](https://console.firebase.google.com/).
    *   Click **Add project** -> Name it `roboquest-academy`.
    *   Disable Google Analytics (optional, simpler for now) -> **Create project**.

2.  **Add Web App**:
    *   Click the **Web icon (`</>`)** on the dashboard.
    *   Nickname: `RoboQuest Web`.
    *   **Uncheck** "Also set up Firebase Hosting" (we are using Netlify).
    *   Click **Register app**.

3.  **Get Config**:
    *   Copy the `firebaseConfig` object (apiKey, authDomain, etc.).
    *   Open `src/services/firebase.ts` in your editor.
    *   Replace the placeholder values with your real keys.

4.  **Enable Authentication**:
    *   Go to **Build** -> **Authentication** in the sidebar.
    *   Click **Get Started**.
    *   Select **Google** -> **Enable**.
    *   Select a support email -> **Save**.

5.  **Enable Firestore (Database)**:
    *   Go to **Build** -> **Firestore Database**.
    *   Click **Create Database**.
    *   Select a location (e.g., `nam5 (us-central)`).
    *   Start in **Test Mode** (allows read/write for 30 days) -> **Create**.

## 3. Netlify (Hosting)
1.  **Import Project**:
    *   Go to [netlify.com](https://www.netlify.com/) and log in.
    *   Click **Add new site** -> **Import an existing project**.
    *   Select **GitHub**.
    *   Authorize Netlify if asked, then search for `roboquest-academy` and select it.

2.  **Configure Build**:
    *   **Build command**: `npm run build` (should be auto-detected).
    *   **Publish directory**: `dist` (should be auto-detected).
    *   Click **Deploy site**.

3.  **Fix Routing (Important)**:
    *   Since this is a Single Page App (SPA), refreshing on a subpage (like `/login`) might give a 404.
    *   Create a file named `_redirects` (no extension) in your `public/` folder with this content:
        ```
        /*  /index.html  200
        ```
    *   Commit and push this change to GitHub to trigger a redeploy.
