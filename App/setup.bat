@echo off
echo === ASL Glove Setup (Admin Mode) ===
echo.

cd /d C:\Users\gunne\AslGlove\App

REM Clean up any broken venv
if exist venv (
    echo Removing broken virtual environment...
    takeown /f venv /r /d y >nul 2>&1
    icacls venv /grant "%username%":F /t /q >nul 2>&1
    rmdir /s /q venv
)

REM Create fresh virtual environment
echo Creating new virtual environment...
python -m venv venv

REM Activate it
call venv\Scripts\activate.bat

REM Install packages
python -m pip install --upgrade pip
pip install bleak matplotlib numpy scipy

echo.
echo === Setup Complete! ===
echo Virtual environment created successfully.
echo.
echo You can now use run.bat normally (no admin needed)
pause
