# ASL Glove IMU Data Pipeline

Real-time IMU data processing and visualization for ASL gesture recognition.

## Setup

### Windows
#### First Time running 
```bash
# Go to main directory, and run setup.bat in admin mode
cd App
set-executionpolicy remotesigned
venv\Scripts\activate.bat
```
#### Running the program
```bash
cmd 
cd App
venv\Scripts\activate.bat #Only have to do on first run
python main.py # Will record the data for later playback, no graphing
python main.py --playback recording_[timestamp] # Will playback and graph it
```

## Linux / Mac TBD 
source venv/bin/activate

## Commands

### Mode Presets

```bash
# Position tracking only (default)
python main.py

# Orientation tracking only
python main.py --mode madgwick

# Raw and converted sensor comparison
python main.py --mode sensor

# Full pipeline (converted sensors + orientation + position)
python main.py --mode debug

# Everything (raw sensors + converted + orientation + position)
python main.py --mode all

# Custom selection
python main.py --mode graph converted madgwick
```

### Performance

```bash
# Update plot every 10 packets (faster, less smooth)
python main.py --update 10

# Update plot every packet (slower, very smooth)
python main.py --update 1
```

### Combined Options

```bash
# Full debug mode with recording
python main.py --mode debug --update 10 --record

# Quick orientation check with minimal logging
python main.py --mode madgwick --update 5 --quiet

# Performance mode
python main.py --mode position --update 20 --max-points 100
```
### Recording and Playback
```
# Record a live session (saves on exit)
python main.py --record

# Save with custom name
python main.py --record --output my_recording.pkl

# Record in JSON format (human-readable)
python main.py --record --output data.json


# Play back at original speed
python main.py --playback recording_20241111_120000.pkl

# Play back at 2x speed
python main.py --playback recording.pkl --speed 2.0

# Play back as fast as possible
python main.py --playback recording.pkl --fast

# Play back with different visualization
python main.py --playback recording.pkl --mode all


# View recording info
python recording_info.py recording.pkl

# View multiple files
python recording_info.py *.pkl
```

## Help

```bash
# Show all options
python main.py --help
```

## Stopping

Press `Ctrl+C` to stop gracefully

## Examples workflow
```
# 1. Record a session
python main.py --mode debug --record --output session1.pkl

# 2. Check what's in it
python recording_info.py session1.pkl

# 3. Analyze it with different mode
python main.py --playback session1.pkl --mode all --speed 2.0

# 4. Convert to JSON for sharing
python main.py --playback session1.pkl --record --output session1.json
```

