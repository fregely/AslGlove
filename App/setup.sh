#!/bin/bash
echo "=== ASL Glove Setup (Linux/Mac) ==="

# Check Python
if ! command -v python3 &> /dev/null; then
    echo "Error: Python 3 not found!"
    exit 1   
fi

# Create and activate virtual environment
echo "Creating virtual environment..."
python3 -m venv venv
source venv/bin/activate

# Install dependencies
echo "Installing dependencies..."
pip install --upgrade pip
pip install -r requirements.txt


deactivate
