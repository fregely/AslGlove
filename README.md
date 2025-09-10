# ASL Glove Project -- Environment Setup Guide :)
This guide walks you through setting up the Python development environment for the ASL Glove project. It uses a virtual environment to isolate dependencies like OpenCV and ensure everything runs consistently across machines.

Requirements
	•	Python 3.9 or higher (recommended)
	•	Git
	•	VS Code (recommended)
	•	macOS or Linux (Windows should work but steps may differ slightly)

 1. Clone the Repository
    ```bash
    git clone <your-repo-url>
    cd AslGlove
    ```
 2. Create and Activate the Virtual Envrionment
    ```bash
    python3 -m venv .venv
    source .venv/bin/activate
    ```
    You should see (.venv) at the beginning of your terminal prompt when activated.
 3. Install Project Dependencies
    ```bash
    pip install -r requirements.txt
    ```
    If you don't see requirements.txt, you can manually install:
    ```bash
    pip install opencv-python numpy
    ```
 4. VS Code Setup
    If you're using VS Code:
    	•	Press Cmd+Shift+P (or Ctrl+Shift+P on Windows)
    	•	Select Python: Select Interpreter
    	•	Choose the one that says .venv

    Now you can run scripts directly with the Run ▶️ button or F5.


openCV blob detection guide 
https://opencv.org/blog/blob-detection-using-opencv/
^ gives both python and C++, I (david) would prefer to use C++ just to keep all the programs 
in c/c++ but if your more comfortable with python you can us that instead

---
## Git Setup (First Time)  

### Install Git  
1. Download Git for Windows: [https://git-scm.com/download/win](https://git-scm.com/download/win)  
2. Install with default settings.  

### Open a Terminal on Windows  
- Press `Windows Key` → type **"Command Prompt"** or **"PowerShell"**.  
- Or open **Git Bash** (installed with Git).  

### Configure Git (first time only)  
Set your name and email (these appear in commits):  
```bash
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"
```

## Generating an SSH Key (to connect to GitHub securely)

1. In your terminal, run:
```bash
ssh-keygen -t rsa -b 4096 -C "your.email@example.com"
```
2. When asked for passphrase press `Enter` to skip
3. Copy your public key to the clipboard
```bash
cat ~/.ssh/id_rsa.pub
```

4. Go to GitHub → Settings → SSH and GPG keys → New SSH key.
5. Paste your key and save.

## Pushing changes 
1. Open folder in terminal
```bash
mv AslGlove
```
2. Go to new dev branch
```bash
git checkout -b "dev/yourName"
```
3. Add all changed file to working directory
```bash
git add .
```
4. Commit changes
```bash
git commit -m "description of changes"
``` 
5. Push changes to repo
```bash
git push
```
6. Open github web browser and make a Pull request

## Git Basic Commands

### Clone a Repository (download project for the first time)
```bash
git clone git@github.com:<username>/<repo-name>.git
cd <repo-name>
```
### Check status (see changes in your files)
```bash
git status
```
### Add files (stage changes for commit)
```bash
git add <filename>    # add one file
git add .             # add everything
```
### Commit changes (save a snapshot locally)
```bash
git commit -m "Explain what you changed"
```
### Push changes (upload your work to GitHub)
```bash
git push origin main
```
### Fetch (see if there are new changes, but don’t merge them yet)
```bash
git fetch origin
```
### Merge (combine changes from others into your branch)
```bash
git merge origin/main
```
### View commit history
```bash
git log --oneline
```
### Undo local changes to a file (irreversible):
```bash
git checkout -- <filename>
```
--- 

## Setting up Rtos Environment TODO
https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/linux-macos-setup.html






