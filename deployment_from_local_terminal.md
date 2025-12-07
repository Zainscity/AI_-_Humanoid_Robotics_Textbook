It seems you are still encountering the "dubious ownership" error. This is a security feature in Git that is common when using Windows Subsystem for Linux (WSL), and it can sometimes be persistent.

To work around this, let's try a more robust approach where you run the Git commands from your own local terminal, outside of this environment.

Please follow these steps **in your local terminal**:

1.  **Ensure you are in the project root directory**:
    ```bash
    cd /home/zain/code/UsedGemini/Hackathon/humanoid_robotics_book
    ```

2.  **Clean and rebuild your Docusaurus project**:
    ```bash
    npm run clear && npm run build
    ```

3.  **Navigate into the `build` directory**:
    ```bash
    cd build
    ```

4.  **Initialize a new Git repository in the `build` directory**:
    ```bash
    git init
    ```

5.  **Add all generated files and commit them**:
    ```bash
    git add -A
    git commit -m "Deploy website to GitHub Pages"
    ```

6.  **Add the remote repository**:
    ```bash
    git remote add origin https://github.com/zainscity/humanoid_robotics_book.git
    ```
    **NOTE**: If you get an error that `origin` already exists, run `git remote remove origin` first, and then run the command again.

7.  **Force push the `build` directory contents to the `gh-pages` branch**:
    ```bash
    git push -f origin master:gh-pages
    ```
    When prompted, enter your GitHub username (`zainscity`) and use your **Personal Access Token (PAT)** as your password.

8.  **Navigate back to your project root**:
    ```bash
    cd ..
    ```

After completing these steps, your Docusaurus website should be deployed to `https://zainscity.github.io/humanoid_robotics_book/`. It may take a few minutes for GitHub Pages to update.
