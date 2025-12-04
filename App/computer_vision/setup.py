#!/usr/bin/env python3
"""
Setup Verification Script
Checks if all files are properly configured before running main.py

Usage:
    python verify_setup.py
"""

import os
import sys
import json

def check_file_exists(filename, required=True):
    """Check if a file exists."""
    exists = os.path.exists(filename)
    status = "✅" if exists else ("❌" if required else "⚠️ ")
    print(f"{status} {filename}: {'Found' if exists else 'Missing'}")
    return exists

def check_finger_map():
    """Validate finger_map.json structure."""
    if not os.path.exists("finger_map.json"):
        return False
    
    try:
        with open("finger_map.json", 'r') as f:
            data = json.load(f)
        
        # Check for all 5 fingers
        expected_leds = ['1', '3', '20', '7', '6']
        expected_fingers = ['thumb', 'index', 'middle', 'ring', 'pinky']
        
        found_fingers = []
        for led_str, info in data.items():
            finger = info.get('finger')
            center = info.get('center')
            
            if finger and center and len(center) == 2:
                found_fingers.append(finger)
            else:
                print(f"   ⚠️  LED {led_str}: Incomplete data")
        
        missing = set(expected_fingers) - set(found_fingers)
        if missing:
            print(f"   ❌ Missing fingers: {missing}")
            return False
        else:
            print(f"   ✅ All 5 fingers present: {found_fingers}")
            return True
            
    except Exception as e:
        print(f"   ❌ Error reading finger_map.json: {e}")
        return False

def check_control_py():
    """Check if control.py has the improved version."""
    if not os.path.exists("imu_processing/control.py"):
        print("❌ imu_processing/control.py: Missing")
        return False
    
    with open("imu_processing/control.py", 'r') as f:
        content = f.read()
    
    # Check for key improvements
    checks = {
        'initialize_channel_position': 'initialize_channel_position' in content,
        '_load_initial_positions': '_load_initial_positions' in content,
        'first_position_received': 'first_position_received' in content,
    }
    
    all_good = all(checks.values())
    status = "✅" if all_good else "❌"
    print(f"{status} imu_processing/control.py: {'Updated' if all_good else 'Needs update'}")
    
    if not all_good:
        print("   Missing improvements:")
        for check, passed in checks.items():
            if not passed:
                print(f"     ❌ {check}")
    
    return all_good

def check_main_py():
    """Check if main.py has proper initialization order."""
    if not os.path.exists("main.py"):
        print("❌ main.py: Missing")
        return False
    
    with open("main.py", 'r') as f:
        lines = f.readlines()
    
    # Find key lines
    logger_setup = None
    control_import = None
    control_create_early = None
    control_create_main = None
    
    for i, line in enumerate(lines):
        if 'logger = logging.getLogger(__name__)' in line:
            if logger_setup is None:
                logger_setup = i
        if 'from imu_processing import' in line:
            control_import = i
        if 'control = Control()' in line and i < 200:  # Early in file
            control_create_early = i
        if 'control = Control(kp=args.pid_kp' in line:
            control_create_main = i
    
    issues = []
    
    # Check 1: Logger should be set up early
    if logger_setup is None:
        issues.append("Logger not initialized")
    elif logger_setup > 100:
        issues.append(f"Logger initialized too late (line {logger_setup})")
    
    # Check 2: Control should NOT be created before imports
    if control_create_early and control_import and control_create_early < control_import:
        issues.append(f"Control created before imports (line {control_create_early} < {control_import})")
    
    # Check 3: Control should be created in main()
    if control_create_main is None:
        issues.append("Control not created in main()")
    
    if issues:
        print(f"❌ main.py: Has issues")
        for issue in issues:
            print(f"     ❌ {issue}")
        return False
    else:
        print(f"✅ main.py: Properly configured")
        return True

def main():
    print("="*70)
    print("🔍 SETUP VERIFICATION")
    print("="*70)
    print()
    
    print("📁 File Check:")
    print("-" * 70)
    has_finger_map = check_file_exists("finger_map.json", required=True)
    has_open_hand = check_file_exists("open_hand_calibration.json", required=False)
    has_control = check_file_exists("imu_processing/control.py", required=True)
    has_main = check_file_exists("main.py", required=True)
    print()
    
    print("🔬 Content Check:")
    print("-" * 70)
    finger_map_valid = check_finger_map() if has_finger_map else False
    control_updated = check_control_py() if has_control else False
    main_correct = check_main_py() if has_main else False
    print()
    
    print("="*70)
    print("📊 SUMMARY")
    print("="*70)
    
    all_checks = [
        ("finger_map.json exists", has_finger_map),
        ("finger_map.json valid", finger_map_valid),
        ("control.py updated", control_updated),
        ("main.py correct", main_correct),
    ]
    
    passed = sum(1 for _, status in all_checks if status)
    total = len(all_checks)
    
    for check, status in all_checks:
        symbol = "✅" if status else "❌"
        print(f"  {symbol} {check}")
    
    print()
    print(f"Score: {passed}/{total} checks passed")
    print("="*70)
    print()
    
    if passed == total:
        print("✅ ALL CHECKS PASSED!")
        print("   Ready to run: python main.py")
        return 0
    else:
        print("❌ SOME CHECKS FAILED")
        print("   Fix the issues above before running main.py")
        print()
        print("Quick fixes:")
        if not has_finger_map or not finger_map_valid:
            print("  1. Run: python main.py --calibrate")
        if not control_updated:
            print("  2. Update imu_processing/control.py with improved version")
        if not main_correct:
            print("  3. Fix main.py initialization order")
        return 1

if __name__ == "__main__":
    sys.exit(main())
