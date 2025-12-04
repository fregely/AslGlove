#!/usr/bin/env python3
"""
Calibration verification tool - checks if finger_map.json is valid
and shows what initial positions will be used.

Usage:
    python check_calibration.py
    python check_calibration.py --px-per-mm 10.0
"""

import json
import os
import argparse

def check_calibration(px_per_mm=10.0):
    """Verify calibration file and show resulting positions."""
    
    filename = "finger_map.json"
    
    print("="*70)
    print("🔍 CALIBRATION VERIFICATION")
    print("="*70)
    print()
    
    # Check if file exists
    if not os.path.exists(filename):
        print(f"❌ ERROR: {filename} not found!")
        print("   Run calibration mode first: python main.py --calibrate")
        return False
    
    # Load the file
    try:
        with open(filename, 'r') as f:
            finger_map = json.load(f)
    except Exception as e:
        print(f"❌ ERROR: Failed to load {filename}: {e}")
        return False
    
    print(f"✅ Loaded {filename}")
    print()
    
    # Calculate conversion factor
    px_to_m = 1.0 / (px_per_mm * 1000)
    print(f"📐 Conversion: {px_per_mm} px/mm → {px_to_m:.6f} m/px")
    print()
    
    # Finger to channel mapping (must match control.py)
    led_to_finger = {
        1: "thumb",
        3: "index",
        20: "middle",
        7: "ring",
        6: "pinky"
    }
    
    imu_to_finger = {
        2: "thumb",
        1: "index",
        0: "middle",
        7: "ring",
        6: "pinky"
    }
    
    # Create reverse mapping
    finger_to_channel = {v: k for k, v in imu_to_finger.items()}
    
    print("📍 CALIBRATED POSITIONS:")
    print("-" * 70)
    print(f"{'Finger':<10} {'LED':<6} {'IMU Ch':<8} {'Pixels':<20} {'Meters':<20}")
    print("-" * 70)
    
    all_valid = True
    positions = {}
    
    for led_gpio_str, info in sorted(finger_map.items(), key=lambda x: int(x[0])):
        led_gpio = int(led_gpio_str)
        finger_name = info.get("finger")
        center_px = info.get("center")
        
        if not finger_name or not center_px:
            print(f"❌ LED {led_gpio}: Missing data")
            all_valid = False
            continue
        
        # Get channel
        channel = finger_to_channel.get(finger_name, "?")
        
        # Convert to meters
        x_px, y_px = center_px
        x_m = x_px * px_to_m
        y_m = y_px * px_to_m
        
        positions[finger_name] = (x_m, y_m)
        
        # Format output
        px_str = f"({x_px:>6.1f}, {y_px:>6.1f})"
        m_str = f"({x_m:>+7.4f}, {y_m:>+7.4f})"
        
        print(f"{finger_name:<10} {led_gpio:<6} ch{channel:<6} {px_str:<20} {m_str:<20}")
    
    print("-" * 70)
    print()
    
    # Check for all required fingers
    required_fingers = ["thumb", "index", "middle", "ring", "pinky"]
    missing = [f for f in required_fingers if f not in positions]
    
    if missing:
        print(f"⚠️  WARNING: Missing fingers: {', '.join(missing)}")
        all_valid = False
    
    # Calculate hand dimensions
    if len(positions) >= 2:
        print("📏 HAND DIMENSIONS:")
        print("-" * 70)
        
        # Calculate distances between fingers
        finger_list = list(positions.keys())
        for i, f1 in enumerate(finger_list):
            for f2 in finger_list[i+1:]:
                x1, y1 = positions[f1]
                x2, y2 = positions[f2]
                distance = ((x2-x1)**2 + (y2-y1)**2)**0.5
                print(f"  {f1} ↔ {f2}: {distance*1000:.1f} mm")
        
        print()
    
    # Summary
    print("="*70)
    if all_valid and len(positions) == 5:
        print("✅ CALIBRATION VALID - All 5 fingers calibrated")
        print()
        print("Next steps:")
        print("  1. Run: python main.py")
        print("  2. IMUs will initialize at these positions")
        print("  3. PID will correct drift during operation")
    else:
        print("⚠️  CALIBRATION INCOMPLETE")
        print()
        print("Run calibration again:")
        print("  python main.py --calibrate")
    print("="*70)
    
    return all_valid

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Verify finger_map.json calibration")
    parser.add_argument('--px-per-mm', type=float, default=10.0,
                       help='Pixels per millimeter (default: 10.0)')
    args = parser.parse_args()
    
    check_calibration(px_per_mm=args.px_per_mm)
