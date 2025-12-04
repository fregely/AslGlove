#!/usr/bin/env python3
# pylint: disable=E1101
# mypy: ignore-errors
"""
Real-time letter detection diagnostic tool.
Shows detailed information about why letters are changing rapidly.
"""

import asyncio
import json
from computer_vision.cv import VisionProcessor
from imu_processing.ble_client import BLEClient
from letter_recognizer2 import LetterRecognizer2

class LetterDiagnostic:
    def __init__(self):
        self.recognizer = LetterRecognizer2()
        self.recognizer.verbose_debug = True
        self.recognizer.debug_mode = True
        
        self.last_states = None
        self.last_letter = None
        self.frame_count = 0
        
    async def run(self):
        """Run diagnostic session."""
        
        print("=== ASL Letter Detection Diagnostic ===")
        print("\nThis tool will show:")
        print("  1. Raw letter detections every frame")
        print("  2. Finger states (extended/curled/closed)")
        print("  3. Key normalized distances")
        print("  4. What changed between frames")
        print("\n" + "="*50)
        
        # Load calibration
        if not self.recognizer.load_calibration():
            print("\n❌ No calibration found! Run: python main.py --calibrate")
            return
        
        print("\n✅ Calibration loaded")
        
        # Initialize vision
        vision = VisionProcessor()
        vision.enable_recording = False
        
        # Initialize BLE
        print("\n🔍 Scanning for ASL Glove...")
        client = BLEIMUClient()
        
        try:
            await client.connect()
            print("✅ Connected to glove")
            
            # Start vision
            vision.start()
            await asyncio.sleep(1.0)  # Let camera warm up
            
            print("\n" + "="*50)
            print("🎯 STARTING DIAGNOSTIC - Hold different letters")
            print("="*50 + "\n")
            
            while True:
                self.frame_count += 1
                
                # Get finger positions from vision
                if not vision.finger_positions:
                    await asyncio.sleep(0.05)
                    continue
                
                packet = vision.finger_positions
                
                # Analyze letter
                states = self.recognizer.classify_finger_states(packet)
                if states is None:
                    print("⚠️ No calibration data")
                    await asyncio.sleep(0.1)
                    continue
                
                # Get raw letter
                letter = self.recognizer.letter_from_states(states, packet)
                
                # Get normalized distances
                n = self.recognizer.compute_normalized_distances(packet)
                
                # Check what changed
                state_changed = (states != self.last_states)
                letter_changed = (letter != self.last_letter)
                
                # Only print when something changes or every 20 frames
                if letter_changed or state_changed or self.frame_count % 20 == 0:
                    print(f"\n--- Frame {self.frame_count} ---")
                    
                    if letter_changed:
                        print(f"🔄 LETTER CHANGED: {self.last_letter} → {letter}")
                    else:
                        print(f"📌 Letter: {letter}")
                    
                    print(f"Finger States:")
                    for finger in ["thumb", "index", "middle", "ring", "pinky"]:
                        state = states[finger]
                        # Show what changed
                        if self.last_states and states[finger] != self.last_states[finger]:
                            print(f"  {finger:8s}: {self.last_states[finger]:8s} → {state:8s} ✱")
                        else:
                            print(f"  {finger:8s}: {state}")
                    
                    print(f"\nKey Distances (normalized):")
                    print(f"  Thumb-Index:  {n[('thumb','index')]:.3f}")
                    print(f"  Thumb-Middle: {n[('thumb','middle')]:.3f}")
                    print(f"  Index-Middle: {n[('index','middle')]:.3f}")
                    print(f"  Middle-Ring:  {n[('middle','ring')]:.3f}")
                    print(f"  Ring-Pinky:   {n[('ring','pinky')]:.3f}")
                    print(f"  Finger Spread: {self.recognizer.finger_spread(packet):.3f}")
                    
                    # Show which rule matched (if any)
                    if letter:
                        self._explain_detection(letter, states, n, packet)
                    else:
                        print("\n❓ No letter matched")
                
                self.last_states = states.copy()
                self.last_letter = letter
                
                await asyncio.sleep(0.1)  # 10 Hz
                
        except KeyboardInterrupt:
            print("\n\n👋 Diagnostic stopped")
        finally:
            vision.stop()
            if client.connected:
                await client.disconnect()
    
    def _explain_detection(self, letter, states, n, packet):
        """Explain why a particular letter was detected."""
        print(f"\n✅ Detected '{letter}' because:")
        
        avg_spread = self.recognizer.finger_spread(packet)
        
        if letter == "O":
            thumb_dists = [n[("thumb", f)] for f in ["index", "middle", "ring", "pinky"]]
            avg_thumb = sum(thumb_dists) / len(thumb_dists)
            max_thumb = max(thumb_dists)
            print(f"  - All fingers close to thumb")
            print(f"    Avg distance: {avg_thumb:.3f} < 0.32 ✓")
            print(f"    Max distance: {max_thumb:.3f} < 0.40 ✓")
        
        elif letter == "F":
            print(f"  - Middle, ring, pinky extended")
            print(f"  - Index closed: {states['index']}")
            print(f"  - Thumb-Index distance: {n[('thumb','index')]:.3f} < 0.30 ✓")
        
        elif letter == "W":
            print(f"  - Index, middle, ring extended")
            print(f"  - Pinky curled: {states['pinky']}")
            print(f"  - Index-Middle spread: {n[('index','middle')]:.3f}")
            print(f"  - Middle-Ring spread: {n[('middle','ring')]:.3f}")
        
        elif letter == "V":
            print(f"  - Index & middle extended, others closed")
            print(f"  - Wide V spacing: {n[('index','middle')]:.3f} > 0.50 ✓")
        
        elif letter == "R":
            print(f"  - Index & middle extended, others closed")
            print(f"  - Crossed/close: {n[('index','middle')]:.3f} < 0.30 ✓")
        
        elif letter == "U":
            print(f"  - Index & middle extended, others closed")
            print(f"  - Parallel spacing: {n[('index','middle')]:.3f} between 0.30-0.50 ✓")
        
        elif letter == "L":
            print(f"  - Thumb & index extended, others closed")
            print(f"  - Right angle: {n[('thumb','index')]:.3f} > 0.60 ✓")
        
        elif letter == "B":
            print(f"  - All 4 fingers extended")
            print(f"  - Thumb curled: {states['thumb']}")
            print(f"  - Fingers together: spread={avg_spread:.3f} < 0.35 ✓")
        
        elif letter in ["A", "S", "E", "M", "N"]:
            ti = n[("thumb", "index")]
            print(f"  - Fist with thumb position")
            print(f"  - Thumb-Index: {ti:.3f}")
            if letter == "E":
                print(f"    < 0.22 (very tight) ✓")
            elif letter == "S":
                print(f"    0.22-0.32 (tight) ✓")
            elif letter == "A":
                print(f"    0.32-0.50 (loose) ✓")
            elif letter == "M":
                tp = n[("thumb", "pinky")]
                print(f"    Thumb-Pinky: {tp:.3f} < 0.35 ✓")
            elif letter == "N":
                tm = n[("thumb", "middle")]
                print(f"    Thumb-Middle: {tm:.3f} < 0.35 ✓")


if __name__ == "__main__":
    diagnostic = LetterDiagnostic()
    asyncio.run(diagnostic.run())
