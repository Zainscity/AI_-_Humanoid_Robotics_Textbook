Thank you for clarifying that you are using GitHub Desktop. That changes the process significantly. Please disregard the previous command-line instructions.

Here is a step-by-step guide to deploying your Docusaurus website using GitHub Desktop. The goal is to replace the contents of a special branch called `gh-pages` with the contents of your `build` folder.

### **Step 1: Build Your Docusaurus Website**

First, we need to generate the static website files.
1.  Open a terminal in your project directory (`/home/zain/code/UsedGemini/Hackathon/humanoid_robotics_book`).
2.  Run the build command:
    ```bash
    npm run build
    ```
    This will create a `build` folder in your project directory. This folder contains your complete website.

### **Step 2: Prepare a Clean `gh-pages` Branch**

We will create a clean branch to hold the website files.

1.  Open your repository in **GitHub Desktop**.
2.  Make sure your current branch is `001-humanoid-robotics-book`.
3.  Go to **Branch -> New Branch...**.
4.  Name the new branch `gh-pages`.
5.  Click **"Create Branch"**.
6.  **Publish** this new branch to GitHub by clicking the **"Publish branch"** button at the top.

### **Step 3: Replace Branch Contents with the Build Output**

This is the most important step. We will make the `gh-pages` branch contain *only* the contents of your `build` folder.

1.  In your **file explorer**, navigate to your project directory: `/home/zain/code/UsedGemini/Hackathon/humanoid_robotics_book`.
2.  **Delete everything** in this directory **EXCEPT** for the `.git` folder and the `build` folder. The `.git` folder is hidden, so you may need to show hidden files to see it.
    *   *This sounds scary, but it's okay! We are on the `gh-pages` branch, so your source code is safe on the `001-humanoid-robotics-book` branch.*
3.  Now, open the `build` folder.
4.  **Select all** files and folders inside the `build` folder.
5.  **Copy** them.
6.  Go back up to the root of your project directory.
7.  **Paste** all the files you just copied.

Your project's root directory should now look like the inside of your `build` folder.

### **Step 4: Commit and Push the Website Files**

1.  Go back to **GitHub Desktop**.
2.  You will see a very large number of changes (all the website files being added).
3.  In the commit summary box at the bottom, type a commit message, for example: `Deploy website to gh-pages`.
4.  Click the **"Commit to gh-pages"** button.
5.  Finally, click the **"Push origin"** button at the top to send your new website to GitHub.

### **Step 5: Configure GitHub Pages**

1.  Go to your repository on the GitHub website: `https://github.com/zainscity/humanoid_robotics_book`.
2.  Go to **Settings -> Pages**.
3.  Under "Branch", select `gh-pages` and `/ (root)` as the folder.
4.  Click **"Save"**.

It may take a few minutes for GitHub to deploy your site. You should then be able to see it live at: `https://zainscity.github.io/humanoid_robotics_book/`
