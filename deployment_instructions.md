To deploy your Docusaurus website to GitHub Pages, please follow these steps using your GitHub Personal Access Token (PAT):

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

5.  **Add all generated files, commit them, and add the remote repository**:
    ```bash
    git add -A
    git commit -m "Deploy Docusaurus to GitHub Pages"
    git remote add origin https://zainscity.github.com/zainscity/humanoid_robotics_book.git
    ```
    **NOTE**: You might need to remove an existing `origin` first if you get an error: `git remote remove origin`

6.  **Force push the `build` directory contents to the `gh-pages` branch**:
    ```bash
    git push -f origin master:gh-pages
    ```
    When prompted for your username, enter `zainscity`.
    When prompted for your password, **paste your GitHub Personal Access Token (PAT)**.

7.  **Navigate back to your project root**:
    ```bash
    cd ..
    ```

After completing these steps, your Docusaurus website should be deployed to `https://zainscity.github.io/humanoid_robotics_book/`. It may take a few minutes for GitHub Pages to update.
