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

## Pushing changes 
1. Open folder in terminal
```bash
cd AslGlove
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
git push --set-upstream origin dev/yourName
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

flashing 
get_idf
idf.py set-target esp32c3
idf.py fullclean
idf.py build //dont really need
idf.py -p /dev/ttyUSB0 flash 
idf.py -p /dev/ttyUSB0 monitor


## Setting up Rtos Environment TODO
https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/linux-macos-setup.html
get_idf - once you have it in that one file that is for sourcing
idf.py set-target esp32c3 #need to install this keychain seperately
https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32c3/esp32-c3-devkitm-1/user_guide.html
ls -l /dev/ttyUSB*
lsusb  -- for usb 
https://www.espressif.com/sites/default/files/documentation/esp32-c3_datasheet_en.pdf

Reading from bluetooth gatt 

bluetoothctl
connect 18:8B:0E:AE:BF:CE
menu gatt
list-attributes

look for the service
/service0028/char0029
UUID: 00002a37-0000-1000-8000-00805f9b34fb
Heart Rate Measurement

select-attribute 00002a37-0000-1000-8000-00805f9b34fb

notify on

https://www.freertos.org/Documentation/01-FreeRTOS-quick-start/01-Beginners-guide/00-Overview
