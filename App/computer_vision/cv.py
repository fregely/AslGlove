# cv.py - PRODUCTION VERSION with Brightness Change Detection
# Works for both letter recognition AND calibration modes

import cv2
import numpy as np
import asyncio
import time
import json
import csv
from datetime import datetime
from collections import deque, defaultdict

SERVICE_UUID = "ffeeddcc-bbaa-0011-2233-445566778899"
IMU_DATA_UUID = "c4e7a180-7b2f-4c95-bfc5-1d5c62123456"
LED_STATE_UUID = "01234567-89ab-cdef-0123-456789abcdef"

LED_NOTIFY_UUID = LED_STATE_UUID
LED_WRITE_UUID = LED_STATE_UUID

CMD_START = bytes([1])
CMD_NEXT = bytes([2])

class VisionProcessor:
    """
    Production CV system with brightness change detection.
    
    Key features:
    - Detects LEDs by brightness CHANGE (eliminates ambient light)
    - Temporal validation (requires consistency across frames)
    - Works in both normal and calibration modes
    - Interactive threshold tuning
    """
    
    def __init__(self, client, record=False, thresh=240, min_area=50, 
                 max_area=400, min_circ=0.70, pixel_per_mm=1.0, 
                 calibration_mode=False):
        self.client = client
        self.current_led = -1
        self.ready_flag = False
        self.record = record
        self.calibration_mode = calibration_mode
        
        # BRIGHTNESS CHANGE DETECTION parameters
        self.BRIGHTNESS_THRESH = thresh  # Absolute brightness threshold for LEDs
        self.MIN_CHANGE_INTENSITY = 80   # Minimum brightness increase to count as "LED turned on"
        self.kernel = np.ones((3,3), np.uint8)
        self.MIN_AREA = min_area
        self.MAX_AREA = max_area
        self.MIN_CIRC = min_circ
        
        print(f"🎯 BRIGHTNESS CHANGE DETECTION:")
        print(f"   Brightness threshold: {self.BRIGHTNESS_THRESH}")
        print(f"   Min change: {self.MIN_CHANGE_INTENSITY}")
        print(f"   Blob area: {self.MIN_AREA}-{self.MAX_AREA}px")
        print(f"   Circularity: {self.MIN_CIRC}+")
        
        # Track previous frame for change detection
        self.prev_gray = None
        self.prev_bright_mask = None
        
        # Temporal validation (requires stability over multiple frames)
        self.detection_history = defaultdict(lambda: deque(maxlen=3))
        self.REQUIRED_CONSECUTIVE = 1
        self.POSITION_STABILITY_THRESHOLD = 10  # pixels
        
        # Stats
        self.stats = {
            'total': 0,
            'accepted': 0,
            'rejected_no_change': 0,
            'rejected_temporal': 0,
            'rejected_quality': 0,
            'no_blobs': 0
        }
        
        # Results
        self.last_blob_centers = []
        self.last_timestamp = None
        self.last_packet = {}
        
        # LED offset
        self.LED_OFFSET_MM = 0
        self.PX_PER_MM = pixel_per_mm
        self.LED_OFFSET_PX = (0, int(self.LED_OFFSET_MM * self.PX_PER_MM))
        
        # Finger mappings
        self.led_to_finger = {
            4: "thumb",
            0: "index",
            1: "middle",
            2: "ring",
            3: "pinky"
        }
        
        self.tracked_fingers = {}
        self.finger_positions = {}
        self.current_letter = None
        
        # CSV
        self.csv_file = None
        self.csv_writer = None
        
        self.load_finger_map("finger_map.json")
    
    def load_finger_map(self, filename):
        """Load finger calibration map."""
        try:
            with open(filename, "r") as f:
                self.finger_map = json.load(f)
            print(f"✅ Loaded {filename}")
        except:
            self.finger_map = {}
            print(f"⚠️  No {filename}")
    
    def _temporal_validation(self, led_index, cx, cy):
        """
        Validate detection is stable across multiple consecutive frames.
        Returns (is_valid, reason_string)
        """
        history = self.detection_history[led_index]
        history.append((cx, cy))
        
        if len(history) < self.REQUIRED_CONSECUTIVE:
            return False, f"Need {self.REQUIRED_CONSECUTIVE} frames (have {len(history)})"
        
        # Check if recent positions are stable
        recent = list(history)[-self.REQUIRED_CONSECUTIVE:]
        positions = np.array(recent)
        
        std_x = np.std(positions[:, 0])
        std_y = np.std(positions[:, 1])
        max_std = max(std_x, std_y)
        
        if max_std < self.POSITION_STABILITY_THRESHOLD:
            return True, f"Stable(std={max_std:.1f}px)"
        else:
            return False, f"Unstable(std={max_std:.1f}px)"
    
    def _calculate_blob_quality(self, area, circ, brightness, is_new_change):
        """
        Calculate quality score 0-1.
        
        Priority scoring:
        1. Is it a NEW bright change? (most important for change detection)
        2. Is it bright enough? (LEDs are very bright)
        3. Is it circular? (LEDs are round)
        4. Is the area reasonable? (not too small/large)
        """
        # Area score - prefer 150-500 pixels
        if 150 <= area <= 500:
            area_score = 1.0
        elif area < 150:
            area_score = area / 150
        else:
            area_score = max(0.2, 1.0 - (area - 500) / 1000)
        
        # Circularity - LEDs are round
        circ_score = circ
        
        # Brightness - LEDs are very bright (>220 usually)
        brightness_score = min(1.0, (brightness - 200) / 50.0)  # Normalize 200-250 → 0-1
        brightness_score = max(0, brightness_score)
        
        # NEW CHANGE - heavily prioritize new bright regions
        change_score = 1.0 if is_new_change else 0.4
        
        # Combined - change detection is most important
        return (area_score * 0.15 + 
                circ_score * 0.20 + 
                brightness_score * 0.20 +
                change_score * 0.45)
    
    def handler(self, _ch, data):
        """BLE callback - LED state notification."""
        if len(data) == 1:
            self.current_led = int(data[0])
            self.ready_flag = True

    async def start(self):
        """Main vision loop."""
        import sys
        print("="*70, file=sys.stderr, flush=True)
        print("🎬 VISION STARTING - BRIGHTNESS CHANGE MODE", file=sys.stderr, flush=True)
        print(f"   Calibration mode: {self.calibration_mode}", file=sys.stderr, flush=True)
        print("="*70, file=sys.stderr, flush=True)
        
        # Camera setup
        cap = cv2.VideoCapture(2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        cap.set(cv2.CAP_PROP_FPS, 60)

        if not cap.isOpened():
            print("❌ Camera failed", file=sys.stderr, flush=True)
            return
        
        print("✅ Camera opened", file=sys.stderr, flush=True)
        
        # Warm up
        for i in range(10):
            cap.read()
        
        print("✅ Camera ready", file=sys.stderr, flush=True)
        
        # CSV setup
        if self.record:
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            csv_filename = f"vision_blobs_{ts}.csv"
            self.csv_file = open(csv_filename, "w", newline="")
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow([
                "frame_ts", "led_index", "cx_px", "cy_px",
                "area", "circ", "quality", "brightness", 
                "is_new_change", "temporal_valid", "accepted"
            ])
            print(f"💾 CSV: {csv_filename}")
        
        # Start LED cycling (normal mode only)
        if not self.calibration_mode:
            await self.client.write_gatt_char(LED_WRITE_UUID, CMD_START, response=False)
            print("✅ LED cycling started", file=sys.stderr, flush=True)
        
        prev_time = time.time()
        
        print("🎬 Main loop starting...", file=sys.stderr, flush=True)
        
        try:
            while True:
                # Wait for LED ready
                if not self.calibration_mode:
                    while not self.ready_flag:
                        await asyncio.sleep(0.0005)
                    self.ready_flag = False
                else:
                    await asyncio.sleep(0)
                
                # Capture frame
                ret, frame = cap.read()
                if not ret:
                    continue
                
                self.stats['total'] += 1
                frame_ts = time.time()
                
                # Convert to grayscale
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                blurred = cv2.GaussianBlur(gray, (5, 5), 0)
                
                # === BRIGHTNESS CHANGE DETECTION ===
                
                # 1. Find all currently bright pixels
                _, current_bright = cv2.threshold(blurred, self.BRIGHTNESS_THRESH, 255, cv2.THRESH_BINARY)
                
                # 2. Find NEW bright pixels (change detection)
                if self.prev_bright_mask is not None and not self.calibration_mode:
                    # Pixels that are bright NOW but were NOT bright before
                    new_bright = cv2.bitwise_and(
                        current_bright,
                        cv2.bitwise_not(self.prev_bright_mask)
                    )
                else:
                    # First frame or calibration mode - treat all bright as "new"
                    new_bright = current_bright.copy()
                
                # Update previous mask
                self.prev_bright_mask = current_bright.copy()
                
                # 3. Clean up masks
                new_bright = cv2.morphologyEx(new_bright, cv2.MORPH_OPEN, self.kernel)
                new_bright = cv2.morphologyEx(new_bright, cv2.MORPH_CLOSE, self.kernel, iterations=2)
                
                current_bright_clean = cv2.morphologyEx(current_bright, cv2.MORPH_OPEN, self.kernel)
                current_bright_clean = cv2.morphologyEx(current_bright_clean, cv2.MORPH_CLOSE, self.kernel, iterations=2)
                
                # 4. Find contours in BOTH masks
                new_contours, _ = cv2.findContours(new_bright, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                all_bright_contours, _ = cv2.findContours(current_bright_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
                # 5. Process candidates
                candidates = []
                
                # Priority 1: NEW bright regions (LED just turned on!)
                for c in new_contours:
                    area = cv2.contourArea(c)
                    if not (self.MIN_AREA < area < self.MAX_AREA):
                        continue

                    per = cv2.arcLength(c, True)
                    circ = 4*np.pi*area/(per*per+1e-9)
                    if circ < self.MIN_CIRC:
                        continue
                    
                    M = cv2.moments(c)
                    if M["m00"] == 0:
                        continue
                    
                    raw_cx = int(M["m10"] / M["m00"])
                    raw_cy = int(M["m01"] / M["m00"])

                    dx, dy = self.LED_OFFSET_PX
                    cx = int(raw_cx + dx)
                    cy = int(raw_cy + dy)
                    
                    # Get brightness
                    mask_region = np.zeros_like(blurred)
                    cv2.drawContours(mask_region, [c], -1, 255, -1)
                    mean_brightness = cv2.mean(blurred, mask=mask_region)[0]
                    
                    quality = self._calculate_blob_quality(area, circ, mean_brightness, is_new_change=True)
                    
                    candidates.append({
                        "center": (cx, cy),
                        "raw_center": (raw_cx, raw_cy),
                        "area": area,
                        "circ": circ,
                        "quality": quality,
                        "brightness": mean_brightness,
                        "is_new_change": True,
                        "contour": c
                    })
                
                # Priority 2: Existing bright regions (fallback if no new changes detected)
                for c in all_bright_contours:
                    area = cv2.contourArea(c)
                    if not (self.MIN_AREA < area < self.MAX_AREA):
                        continue

                    per = cv2.arcLength(c, True)
                    circ = 4*np.pi*area/(per*per+1e-9)
                    if circ < self.MIN_CIRC:
                        continue
                    
                    M = cv2.moments(c)
                    if M["m00"] == 0:
                        continue
                    
                    raw_cx = int(M["m10"] / M["m00"])
                    raw_cy = int(M["m01"] / M["m00"])

                    dx, dy = self.LED_OFFSET_PX
                    cx = int(raw_cx + dx)
                    cy = int(raw_cy + dy)
                    
                    # Skip if already in candidates from new_bright
                    is_duplicate = any(
                        abs(cand["center"][0] - cx) < 20 and 
                        abs(cand["center"][1] - cy) < 20 
                        for cand in candidates
                    )
                    if is_duplicate:
                        continue
                    
                    # Get brightness
                    mask_region = np.zeros_like(blurred)
                    cv2.drawContours(mask_region, [c], -1, 255, -1)
                    mean_brightness = cv2.mean(blurred, mask=mask_region)[0]
                    
                    quality = self._calculate_blob_quality(area, circ, mean_brightness, is_new_change=False)
                    
                    candidates.append({
                        "center": (cx, cy),
                        "raw_center": (raw_cx, raw_cy),
                        "area": area,
                        "circ": circ,
                        "quality": quality,
                        "brightness": mean_brightness,
                        "is_new_change": False,
                        "contour": c
                    })
                
                # === SELECT BEST BLOB WITH TEMPORAL VALIDATION ===
                cv_finger = self.led_to_finger.get(self.current_led)
                accepted = None
                reason = "no_candidates"
                
                if candidates:
                    # Sort by quality (NEW changes rank highest)
                    candidates.sort(key=lambda b: b['quality'], reverse=True)
                    best = candidates[0]
                    cx, cy = best["center"]
                    
                    # Quality threshold
                    if best["quality"] < 0.5:
                        reason = f"Q={best['quality']:.2f}"
                        self.stats['rejected_quality'] += 1
                    else:
                        # Temporal validation
                        temporal_valid, temporal_reason = self._temporal_validation(self.current_led, cx, cy)
                        
                        if temporal_valid:
                            accepted = best
                            self.stats['accepted'] += 1
                        else:
                            reason = temporal_reason
                            self.stats['rejected_temporal'] += 1
                else:
                    self.stats['no_blobs'] += 1
                
                # === VISUALIZATION ===
                display = frame.copy()
                
                # Draw all candidates with color coding
                for cand in candidates:
                    cx, cy = cand["center"]
                    x, y, w, h = cv2.boundingRect(cand["contour"])
                    
                    if cand == accepted:
                        color = (0, 255, 0)  # GREEN = accepted
                        thickness = 3
                        label_color = (0, 255, 0)
                    elif cand["is_new_change"]:
                        color = (255, 165, 0)  # ORANGE = new change but rejected
                        thickness = 2
                        label_color = (255, 165, 0)
                    else:
                        color = (128, 128, 128)  # GRAY = existing bright blob
                        thickness = 1
                        label_color = (128, 128, 128)
                    
                    cv2.rectangle(display, (x, y), (x + w, y + h), color, thickness)
                    cv2.circle(display, (cx, cy), 5, color, -1)
                    
                    # Show metrics
                    change_marker = "★" if cand["is_new_change"] else "○"
                    info = f"{change_marker} Q:{cand['quality']:.2f} B:{cand['brightness']:.0f}"
                    cv2.putText(display, info, (cx + 8, cy - 8),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, label_color, 1)
                
                # Show rejection reason
                if not accepted and candidates:
                    best = candidates[0]
                    cx, cy = best["center"]
                    cv2.putText(display, f"REJECT: {reason}", (cx + 8, cy + 18),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                
                # Store results
                blob_centers = [accepted["center"]] if accepted else []
                self.last_blob_centers = blob_centers
                self.last_timestamp = frame_ts
                
                if blob_centers:
                    self._assign_fingers(blob_centers)
                
                self.last_packet = {
                    "cv_timestamp": frame_ts,
                    "led_index": self.current_led,
                    "blob_centers": blob_centers,
                    "num_blobs": len(blob_centers)
                }
                
                # CSV logging
                if self.csv_writer and accepted:
                    self.csv_writer.writerow([
                        frame_ts, self.current_led,
                        accepted["center"][0], accepted["center"][1],
                        round(accepted['area']), round(accepted['circ'], 2),
                        round(accepted['quality'], 2), round(accepted['brightness']),
                        accepted['is_new_change'], True, True
                    ])
                # print(f"Frame {self.stats['total']:04d} | LED {self.current_led} | ")
                # === OVERLAYS ===
                now = time.time()
                fps = 1/(now - prev_time) if (now - prev_time) > 0 else 0
                prev_time = now
                
                # LED indicator
                led_color = (0, 255, 0) if blob_centers else (0, 0, 255)
                cv2.putText(display, f"LED:{self.current_led}", (10, 40),
                           cv2.FONT_HERSHEY_SIMPLEX, 1.2, led_color, 3)
                
                # FPS
                cv2.putText(display, f"FPS:{fps:.1f}", (10, 90),
                           cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 0), 2)
                
                # Stats
                if self.stats['total'] > 10:
                    rate = 100 * self.stats['accepted'] / self.stats['total']
                    rate_color = (0, 255, 0) if rate > 70 else (255, 165, 0) if rate > 40 else (0, 0, 255)
                    
                    cv2.putText(display, f"Accept:{rate:.0f}%", (10, 140),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.8, rate_color, 2)
                
                # Finger labels
                for finger, pos in self.finger_positions.items():
                    if pos:
                        x, y = int(pos[0]), int(pos[1])
                        cv2.circle(display, (x, y), 10, (255, 0, 0), -1)
                        cv2.putText(display, finger, (x + 15, y + 5),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                
                # Letter
                if self.current_letter:
                    cv2.putText(display, f"Letter: {self.current_letter}", 
                               (20, display.shape[0] - 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 255, 0), 4)
                
                # Candidate info
                new_count = sum(1 for c in candidates if c["is_new_change"])
                cv2.putText(display, f"Blobs:{len(candidates)} (NEW:{new_count})", 
                           (10, 190),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)
                
                # === DISPLAY ===
                cv2.imshow("ASL Vision", display)
                
                # Optional: Show brightness and change masks
                if not self.calibration_mode:
                    cv2.imshow("Bright Pixels", current_bright_clean)
                    cv2.imshow("NEW Bright (Change)", new_bright)
                
                # Keyboard controls
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    print("\n🛑 User quit")
                    break
                elif key == ord('r'):
                    print("🔄 Reset change detection")
                    self.prev_bright_mask = None
                    self.detection_history.clear()
                elif key == ord(']'):
                    self.BRIGHTNESS_THRESH += 10
                    print(f"📈 Brightness threshold: {self.BRIGHTNESS_THRESH}")
                elif key == ord('['):
                    self.BRIGHTNESS_THRESH = max(150, self.BRIGHTNESS_THRESH - 10)
                    print(f"📉 Brightness threshold: {self.BRIGHTNESS_THRESH}")
                elif key == ord('}'):
                    self.MIN_CHANGE_INTENSITY += 10
                    print(f"📈 Change intensity: {self.MIN_CHANGE_INTENSITY}")
                elif key == ord('{'):
                    self.MIN_CHANGE_INTENSITY = max(30, self.MIN_CHANGE_INTENSITY - 10)
                    print(f"📉 Change intensity: {self.MIN_CHANGE_INTENSITY}")
                
                # Advance LED
                if not self.calibration_mode:
                    await self.client.write_gatt_char(LED_WRITE_UUID, CMD_NEXT, response=False)
                    await asyncio.sleep(0.01)
        
        except KeyboardInterrupt:
            print("\n🛑 Interrupted")
        except Exception as e:
            print(f"\n❌ Error: {e}")
            import traceback
            traceback.print_exc()
        finally:
            cap.release()
            cv2.destroyAllWindows()
            
            if self.csv_file:
                self.csv_file.close()
            
            # Print final stats
            print("\n📊 CV FINAL STATISTICS:")
            print(f"  Total frames: {self.stats['total']}")
            print(f"  Accepted: {self.stats['accepted']}")
            print(f"  Rejected (quality): {self.stats['rejected_quality']}")
            print(f"  Rejected (temporal): {self.stats['rejected_temporal']}")
            print(f"  No blobs: {self.stats['no_blobs']}")
            
            if self.stats['total'] > 0:
                rate = 100 * self.stats['accepted'] / self.stats['total']
                print(f"  Acceptance rate: {rate:.1f}%")
            
            print("📴 Vision stopped")
    
    def _assign_fingers(self, blob_centers):
        """Assign detected blob to finger based on current LED index."""
        if not blob_centers or self.current_led < 0:
            return
        
        cv_finger = self.led_to_finger.get(self.current_led)
        if not cv_finger:
            return
        
        cx, cy = blob_centers[0]
        
        # led_order = [1, 3, 20, 7, 6]


        # # Get finger for this LED frame
        # self.correct_led = led_order(self.current_led)

        # Exponential smoothing
        alpha = 0.3
        prev = self.tracked_fingers.get(cv_finger)
        
        if prev:
            px, py = prev
            smoothed = (alpha * cx + (1-alpha) * px,
                       alpha * cy + (1-alpha) * py)
        else:
            smoothed = (cx, cy)
        
        self.tracked_fingers[cv_finger] = smoothed
        self.finger_positions[cv_finger] = smoothed
    
    def update_finger_positions(self, mapping):
        """Called by main.py to sync positions."""
        self.finger_positions = mapping
    
    def get_packet(self):
        """Return latest CV packet."""
        return self.last_packet
