# cv.py - TUNED differential to only detect BRIGHT LED changes
# The "thermal" effect means it's working, just too sensitive!

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
    """CV with BRIGHTNESS CHANGE DETECTION - Simplest and most robust."""
    
    def __init__(self, client, record=False, thresh=80, min_area=100, 
                 max_area=2000, min_circ=0.70, pixel_per_mm=10.0, 
                 calibration_mode=False):
        self.client = client
        self.current_led = -1
        self.ready_flag = False
        self.record = record
        self.calibration_mode = calibration_mode
        
        # SIMPLIFIED: Only brightness threshold matters
        self.BRIGHTNESS_THRESH = 250  # Adjust based on your LEDs (150-220)
        self.MIN_CHANGE_AREA = 50     # New bright blob must be at least this big
        self.kernel = np.ones((3,3), np.uint8)
        self.MIN_AREA = min_area
        self.MAX_AREA = max_area
        self.MIN_CIRC = min_circ
        
        print(f"🎯 BRIGHTNESS CHANGE DETECTION initialized:")
        print(f"   Brightness threshold: {self.BRIGHTNESS_THRESH}")
        print(f"   Min change area: {self.MIN_CHANGE_AREA} pixels")
        print(f"   Blob area: {self.MIN_AREA}-{self.MAX_AREA} pixels")
        print(f"   Circularity: {self.MIN_CIRC}+")
        
        # Track previous brightness mask
        self.prev_bright_mask = None
        
        # Temporal filtering (keep this - it's still useful)
        self.detection_history = defaultdict(lambda: deque(maxlen=3))
        self.REQUIRED_CONSECUTIVE = 2  # Less strict since we're more confident
        self.POSITION_STABILITY_THRESHOLD = 8  # pixels
        
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
        self.LED_OFFSET_MM = 7.0
        self.PX_PER_MM = pixel_per_mm
        self.LED_OFFSET_PX = (0, int(self.LED_OFFSET_MM * self.PX_PER_MM))
        
        # Mappings
        self.led_to_finger = {
            1: "thumb", 3: "index", 20: "middle", 7: "ring", 6: "pinky"
        }
        
        self.tracked_fingers = {}
        self.finger_positions = {}
        self.current_letter = None
        
        # CSV
        self.csv_file = None
        self.csv_writer = None
        
        self.load_finger_map("finger_map.json")
    
    def load_finger_map(self, filename):
        """Load calibration."""
        try:
            with open(filename, "r") as f:
                self.finger_map = json.load(f)
            print(f"✅ Loaded {filename}")
        except:
            self.finger_map = {}
    
    def _detect_brightness_change(self, current_bright_mask):
        """
        Detect NEW bright regions (LED turning on).
        Returns mask of ONLY the new bright pixels.
        """
        if self.prev_bright_mask is None:
            # First frame - everything is "new"
            self.prev_bright_mask = current_bright_mask.copy()
            return current_bright_mask
        
        # Find pixels that are bright NOW but were NOT bright before
        # This is the LED turning on!
        new_bright_pixels = cv2.bitwise_and(
            current_bright_mask,
            cv2.bitwise_not(self.prev_bright_mask)
        )
        
        # Update previous mask
        self.prev_bright_mask = current_bright_mask.copy()
        
        return new_bright_pixels
    
    def _temporal_validation(self, led_index, cx, cy):
        """Only accept detections stable across multiple frames."""
        history = self.detection_history[led_index]
        history.append((cx, cy))
        
        if len(history) < self.REQUIRED_CONSECUTIVE:
            return False, f"History:{len(history)}/{self.REQUIRED_CONSECUTIVE}"
        
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
        Quality score heavily weighted toward:
        1. Being a NEW bright change (most important!)
        2. High brightness
        3. Good circularity
        4. Reasonable area
        """
        # Area score
        if 150 <= area <= 700:
            area_score = 1.0
        elif area < 150:
            area_score = area / 150
        else:
            area_score = max(0.2, 1.0 - (area - 700) / 1500)
        
        # Circularity score
        circ_score = circ
        
        # Brightness score
        brightness_score = min(1.0, brightness / 230.0)
        
        # NEW CHANGE score (most important!)
        change_score = 1.0 if is_new_change else 0.3
        
        # Combined score - heavily weight change detection
        return (area_score * 0.10 + 
                circ_score * 0.20 + 
                brightness_score * 0.20 + 
                change_score * 0.50)  # 50% weight on being NEW!
    
    def handler(self, _ch, data):
        """BLE callback."""
        if len(data) == 1:
            self.current_led = int(data[0])
            self.ready_flag = True

    async def start(self):
        """Main vision loop with BRIGHTNESS CHANGE detection."""
        import sys
        print("="*70, file=sys.stderr, flush=True)
        print("🎬 VISION STARTING - BRIGHTNESS CHANGE MODE", file=sys.stderr, flush=True)
        print("="*70, file=sys.stderr, flush=True)
        
        # Camera setup
        cap = cv2.VideoCapture(2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        cap.set(cv2.CAP_PROP_FPS, 60)

        if not cap.isOpened():
            print("❌ Camera failed to open", file=sys.stderr, flush=True)
            return
        
        print("✅ Camera opened", file=sys.stderr, flush=True)
        
        # Warm up
        for i in range(10):
            ret, _ = cap.read()
            if not ret:
                print(f"❌ Warmup frame {i} failed", file=sys.stderr, flush=True)
        
        print("✅ Camera warmed up", file=sys.stderr, flush=True)
        
        # CSV
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
        
        # Start LED cycling
        if not self.calibration_mode:
            await self.client.write_gatt_char(LED_WRITE_UUID, CMD_START, response=False)
            print("✅ LED cycling started", file=sys.stderr, flush=True)
        
        prev_time = time.time()
        
        print("🎬 Entering main loop...", file=sys.stderr, flush=True)
        
        try:
            while True:
                # Wait for LED ready
                if not self.calibration_mode:
                    while not self.ready_flag:
                        await asyncio.sleep(0.0005)
                    self.ready_flag = False
                else:
                    await asyncio.sleep(0)
                
                # Capture
                ret, frame = cap.read()
                if not ret:
                    continue
                
                self.stats['total'] += 1
                frame_ts = time.time()
                
                # Process
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                blurred = cv2.GaussianBlur(gray, (5, 5), 0)
                
                # Get brightness mask (all bright pixels)
                _, bright_mask = cv2.threshold(blurred, self.BRIGHTNESS_THRESH, 255, cv2.THRESH_BINARY)
                
                # KEY STEP: Detect CHANGE (new bright regions)
                change_mask = self._detect_brightness_change(bright_mask)
                
                # Clean up change mask
                change_mask = cv2.morphologyEx(change_mask, cv2.MORPH_OPEN, self.kernel)
                change_mask = cv2.morphologyEx(change_mask, cv2.MORPH_CLOSE, self.kernel, iterations=2)
                
                # Find blobs in BOTH masks
                # 1. Blobs in change mask (NEW LEDs - highest priority)
                change_contours, _ = cv2.findContours(change_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
                # 2. Blobs in brightness mask (ALL bright things - lower priority)
                bright_contours, _ = cv2.findContours(bright_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
                # Analyze blobs
                candidates = []
                
                # First, process NEW bright blobs (highest confidence)
                for c in change_contours:
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
                    
                    # Get brightness at this location
                    mask_region = np.zeros_like(blurred)
                    cv2.drawContours(mask_region, [c], -1, 255, -1)
                    mean_brightness = cv2.mean(blurred, mask=mask_region)[0]
                    
                    # Quality score (this is a NEW change - high priority!)
                    quality = self._calculate_blob_quality(area, circ, mean_brightness, is_new_change=True)
                    
                    candidates.append({
                        "center": (cx, cy),
                        "area": area,
                        "circ": circ,
                        "quality": quality,
                        "brightness": mean_brightness,
                        "is_new_change": True,
                        "contour": c
                    })
                
                # Then, add existing bright blobs as fallback (lower priority)
                for c in bright_contours:
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
                    
                    # Skip if we already found this as a "new change"
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
                    
                    # Quality score (NOT a new change - lower priority)
                    quality = self._calculate_blob_quality(area, circ, mean_brightness, is_new_change=False)
                    
                    candidates.append({
                        "center": (cx, cy),
                        "area": area,
                        "circ": circ,
                        "quality": quality,
                        "brightness": mean_brightness,
                        "is_new_change": False,
                        "contour": c
                    })
                
                # SELECT BEST with TEMPORAL VALIDATION
                cv_finger = self.led_to_finger.get(self.current_led)
                accepted = None
                reason = "no_candidates"
                
                if candidates:
                    # Sort by quality (NEW changes will rank highest)
                    candidates.sort(key=lambda b: b['quality'], reverse=True)
                    best = candidates[0]
                    cx, cy = best["center"]
                    
                    # Quality check
                    if best["quality"] < 0.5:
                        reason = f"Q={best['quality']:.2f}<0.5"
                        self.stats['rejected_quality'] += 1
                    elif cv_finger:
                        # Temporal validation
                        temporal_valid, temporal_reason = self._temporal_validation(self.current_led, cx, cy)
                        
                        if temporal_valid:
                            accepted = best
                            self.stats['accepted'] += 1
                        else:
                            reason = f"Temporal:{temporal_reason}"
                            self.stats['rejected_temporal'] += 1
                    else:
                        accepted = best
                        self.stats['accepted'] += 1
                else:
                    self.stats['no_blobs'] += 1
                
                # VISUALIZATION
                display = frame.copy()
                
                # Draw all candidates
                for cand in candidates:
                    cx, cy = cand["center"]
                    x, y, w, h = cv2.boundingRect(cand["contour"])
                    
                    if cand == accepted:
                        color = (0, 255, 0)  # Green = accepted
                        thickness = 3
                    elif cand["is_new_change"]:
                        color = (255, 165, 0)  # Orange = new change but not accepted
                        thickness = 2
                    else:
                        color = (128, 128, 128)  # Gray = existing bright blob
                        thickness = 1
                    
                    cv2.rectangle(display, (x, y), (x + w, y + h), color, thickness)
                    cv2.circle(display, (cx, cy), 4, color, -1)
                    
                    # Show metrics
                    change_str = "NEW" if cand["is_new_change"] else "old"
                    info = f"Q:{cand['quality']:.2f} B:{cand['brightness']:.0f} {change_str}"
                    cv2.putText(display, info, (cx + 5, cy - 5),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1)
                
                # Draw rejection reason
                if not accepted and candidates:
                    best = candidates[0]
                    cx, cy = best["center"]
                    cv2.putText(display, f"REJECT: {reason}", (cx + 5, cy + 15),
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
                    
                    # Show rejection breakdown
                    cv2.putText(display, f"Rej:Q={self.stats['rejected_quality']} "
                                        f"T={self.stats['rejected_temporal']}", 
                               (10, 170),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
                
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
                
                # Candidate count
                new_count = sum(1 for c in candidates if c["is_new_change"])
                cv2.putText(display, f"Blobs:{len(candidates)} (NEW:{new_count})", 
                           (10, 200),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)
                
                # === DISPLAY WINDOWS ===
                cv2.imshow("ASL Vision - Main", display)
                
                # Debug windows
                cv2.imshow("1. Brightness Mask", bright_mask)
                cv2.imshow("2. Change Mask (NEW bright)", change_mask)
                
                # Keyboard controls
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    print("\n🛑 User quit")
                    break
                elif key == ord('r'):
                    print("🔄 Resetting change detection...")
                    self.prev_bright_mask = None
                    self.detection_history.clear()
                elif key == ord(']'):
                    self.BRIGHTNESS_THRESH += 10
                    print(f"📈 Brightness threshold: {self.BRIGHTNESS_THRESH}")
                elif key == ord('['):
                    self.BRIGHTNESS_THRESH = max(150, self.BRIGHTNESS_THRESH - 10)
                    print(f"📉 Brightness threshold: {self.BRIGHTNESS_THRESH}")
                
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
            
            # Final stats
            print("\n📊 FINAL CV STATISTICS (BRIGHTNESS CHANGE):")
            print(f"  Frames: {self.stats['total']}")
            print(f"  Accepted: {self.stats['accepted']}")
            print(f"  Rejected (quality): {self.stats['rejected_quality']}")
            print(f"  Rejected (temporal): {self.stats['rejected_temporal']}")
            print(f"  No blobs: {self.stats['no_blobs']}")
            
            if self.stats['total'] > 0:
                rate = 100 * self.stats['accepted'] / self.stats['total']
                print(f"  Accept rate: {rate:.1f}%")
    
    def _assign_fingers(self, blob_centers):
        """Assign blob to finger."""
        if not blob_centers or self.current_led < 0:
            return
        
        cv_finger = self.led_to_finger.get(self.current_led)
        if not cv_finger:
            return
        
        cx, cy = blob_centers[0]
        
        # Smooth tracking
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
        """External update."""
        self.finger_positions = mapping
    
    def get_packet(self):
        """Get latest packet."""
        return self.last_packet
