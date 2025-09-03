# ASL Glove Project  


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

## Git Guide

### Clone a Repository (download project for the first time)
    git clone git@github.com:<username>/<repo-name>.git
    cd <repo-name>

### Check status (see changes in your files)
    git status

### Add files (stage changes for commit)
    git add <filename>    # add one file
    git add .             # add everything

### Commit changes (save a snapshot locally)
    git commit -m "Explain what you changed"

### Push changes (upload your work to GitHub)
    git push origin main

### Fetch (see if there are new changes, but don’t merge them yet)
    git fetch origin

### Merge (combine changes from others into your branch)
    git merge origin/main

### View commit history
    git log --oneline

### Undo local changes to a file (irreversible):
    git checkout -- <filename>

--- 

## Setting up Rtos Environment 






