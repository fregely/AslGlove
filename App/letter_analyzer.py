#!/usr/bin/env python3
# pylint: disable=E1101
# mypy: ignore-errors
"""
ASL Letter Shape Analyzer

This utility helps you understand how your hand shapes map to letter detection
by showing normalized distances and states for each letter you form.

Usage:
    python letter_analyzer.py

Then form each letter and see the measurements.
"""

import asyncio
import sys
from letter_recognizer2 import LetterRecognizer2
from computer_vision.cv import VisionProcessor
from imu_processing.ble_client import BLEClient

# Constants from main.py
THRESH = 228
MIN_AREA = 50
MAX_AREA = 500
MIN_CIRC = 0.6
PIXEL_PER_MM = 1.0

class LetterAnalyzer:
    """Analyze and display letter shape metrics."""
    
    def __init__(self):
        self.recognizer = LetterRecognizer2()
        self.recognizer.load_calibration()
        
    async def run(self):
        """Run the analyzer."""
        print("="*70)
        print("ASL LETTER SHAPE ANALYZER")
        print("="*70)
        print("\nThis tool shows you the normalized distances and finger states")
        print("for each letter shape you make. Use this to:")
        print("  1. Verify your letter shapes match ASL standards")
        print("  2. Debug recognition issues")
        print("  3. Fine-tune detection thresholds")
        print()
        print("Controls:")
        print("  - Form a letter and hold still")
        print("  - Press SPACE to capture and analyze")
        print("  - Press 'q' to quit")
        print("="*70)
        print()
        
        # Setup BLE and Vision
        client = BLEClient()
        await client.connect()
        await client.start_streaming()
        
        vp = VisionProcessor(
            client.client, 
            record=False, 
            thresh=THRESH,
            min_area=MIN_AREA,
            max_area=MAX_AREA,
            min_circ=MIN_CIRC,
            pixel_per_mm=PIXEL_PER_MM
        )
        vp.load_finger_map("finger_map.json")
        client.set_external_handler(vp.handler)
        
        # Start vision task
        vision_task = asyncio.create_task(vp.start())
        
        print("✅ Connected to ASL Glove")
        print("✅ Vision processor started")
        print()
        print("Form a letter and press SPACE to analyze...")
        print()
        
        try:
            capture_count = 0
            while True:
                await asyncio.sleep(0.1)
                
                # Wait for finger positions
                if not vp.finger_positions or len(vp.finger_positions) < 5:
                    continue
                
                # Check for space bar press (this is simplified - you'd need proper key handling)
                # For now, analyze every 2 seconds automatically
                capture_count += 1
                if capture_count < 20:  # Wait 2 seconds
                    continue
                    
                capture_count = 0
                
                # Analyze current shape
                self.analyze_shape(vp.finger_positions)
                
        except KeyboardInterrupt:
            print("\n👋 Stopping analyzer...")
        finally:
            vision_task.cancel()
            try:
                await vision_task
            except asyncio.CancelledError:
                pass
            await client.stop_streaming()
            await client.disconnect()
    
    def analyze_shape(self, packet):
        """Analyze and display current hand shape."""
        
        # Get finger states
        states = self.recognizer.classify_finger_states(packet)
        if not states:
            print("⚠️ No calibration data - run calibration first")
            return
        
        # Get normalized distances
        n = self.recognizer.compute_normalized_distances(packet)
        if not n:
            return
        
        # Get detected letter
        letter = self.recognizer.letter_from_states(states, packet)
        
        print("\n" + "="*70)
        print(f"DETECTED LETTER: {letter if letter else 'None'}")
        print("="*70)
        
        # Show finger states
        print("\n📋 Finger States:")
        print(f"  Thumb:  {states['thumb']:8s}")
        print(f"  Index:  {states['index']:8s}")
        print(f"  Middle: {states['middle']:8s}")
        print(f"  Ring:   {states['ring']:8s}")
        print(f"  Pinky:  {states['pinky']:8s}")
        
        # Show key distances
        print("\n📏 Key Distances (normalized to open hand):")
        print("  Thumb to Fingers:")
        for finger in ["index", "middle", "ring", "pinky"]:
            dist = n[("thumb", finger)]
            print(f"    T-{finger[0].upper()}: {dist:.3f}", end="")
            if dist < 0.25:
                print("  [CLOSED]")
            elif dist < 0.45:
                print("  [CURLED]")
            elif dist < 0.72:
                print("  [SEMI]")
            else:
                print("  [EXTENDED]")
        
        print("\n  Between Fingers:")
        pairs = [
            ("index", "middle"),
            ("middle", "ring"),
            ("ring", "pinky"),
            ("index", "ring"),
            ("index", "pinky")
        ]
        for f1, f2 in pairs:
            if (f1, f2) in n:
                dist = n[(f1, f2)]
                print(f"    {f1[0].upper()}-{f2[0].upper()}: {dist:.3f}")
        
        # Show finger spread
        spread = self.recognizer.finger_spread(packet)
        print(f"\n  Average finger spread: {spread:.3f}")
        
        # Show 3D positions
        print("\n📍 3D Positions (meters):")
        for finger in ["thumb", "index", "middle", "ring", "pinky"]:
            if finger in packet:
                x, y, z = packet[finger]
                print(f"  {finger:6s}: ({x:6.3f}, {y:6.3f}, {z:6.3f})")
        
        print("\n" + "="*70)
        
        # Suggestions
        if not letter:
            print("\n💡 No letter detected. Try:")
            print("  - Hold your hand more still")
            print("  - Make clearer finger positions")
            print("  - Check if fingers match expected states above")
        
        print()

if __name__ == "__main__":
    analyzer = LetterAnalyzer()
    try:
        asyncio.run(analyzer.run())
    except KeyboardInterrupt:
        print("\n👋 Goodbye!")
