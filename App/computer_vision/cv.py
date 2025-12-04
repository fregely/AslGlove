# cv.py - WITH REGION OF INTEREST (ROI) TRACKING
# Constrains blob detection to hand area for aggressive detection

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
    Vision with ROI constraint - only search for blobs in hand area.
    
    Benefits:
    - Can use MORE AGGRESSIVE detection parameters
    - Eliminates false positives from room lights, windows, etc.
    - Faster processing (smaller search area)
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
        self.BRIGHTNESS_THRESH = thresh
        self.MIN_CHANGE_INTENSITY = 80
        self.kernel = np.ones((3,3), np.uint8)
        self.MIN_AREA = min_area
        self.MAX_AREA = max_area
        self.MIN_CIRC = min_circ
        
        # ========================================
        # ROI TRACKING - NEW!
        # ========================================
        self.roi_enabled = True  # Toggle with 'o' key
        self.roi_bbox = None  # (x, y, w, h) or None
        self.roi_margin = 250  # pixels - generous margin for hand movement
        self.roi_min_fingers = 2  # Need N fingers detected before enabling ROI
        self.roi_update_counter = 0
        self.roi_update_interval = 10  # Update ROI every N frames
        self.roi_auto_reset_threshold = 500  # Reset if no detections for N frames
        self.roi_no_detection_counter = 0
        
        print(f"🎯 ROI TRACKING ENABLED:")
        print(f"   Margin: {self.roi_margin}px")
        print(f"   Min fingers: {self.roi_min_fingers}")
        print(f"   Update interval: {self.roi_update_interval} frames")
        
        # Track previous frame for change detection
        self.prev_gray = None
        self.prev_bright_mask = None
        
        # Temporal validation
        self.detection_history = defaultdict(lambda: deque(maxlen=3))
        self.REQUIRED_CONSECUTIVE = 1
        self.POSITION_STABILITY_THRESHOLD = 10
        
        # Stats
        self.stats = {
            'total': 0,
            'accepted': 0,
            'rejected_no_change': 0,
            'rejected_temporal': 0,
            'rejected_quality': 0,
            'no_blobs': 0,
            'roi_active': 0,  # NEW
            'roi_full_frame': 0  # NEW
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
            3: "ring",
            2: "pinky"
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
    
    def _update_roi_from_fingers(self, frame_width, frame_height):
        """
        Calculate ROI bounding box from current tracked finger positions.
        Returns (x, y, w, h) or None if insufficient data.
        """
        valid_positions = [pos for pos in self.finger_positions.values() if pos]
        
        # Need minimum number of fingers to establish ROI
        if len(valid_positions) < self.roi_min_fingers:
            return None
        
        positions = np.array(valid_positions)
        
        # Find bounding box with generous margin
        x_min = np.min(positions[:, 0]) - self.roi_margin
        y_min = np.min(positions[:, 1]) - self.roi_margin
        x_max = np.max(positions[:, 0]) + self.roi_margin
        y_max = np.max(positions[:, 1]) + self.roi_margin
        
        # Clamp to frame boundaries
        x_min = max(0, int(x_min))
        y_min = max(0, int(y_min))
        x_max = min(frame_width, int(x_max))
        y_max = min(frame_height, int(y_max))
        
        w = x_max - x_min
        h = y_max - y_min
        
        # Sanity check - ROI must be reasonable size
        if w < 100 or h < 100:
            return None
        
        return (x_min, y_min, w, h)
    
    def _apply_roi_mask(self, mask):
        """
        Zero out all pixels OUTSIDE the ROI.
        This constrains blob detection to hand area only.
        """
        if not self.roi_enabled or self.roi_bbox is None:
            return mask  # No ROI constraint
        
        # Create mask with ROI region = 255, rest = 0
        roi_mask = np.zeros_like(mask)
        x, y, w, h = self.roi_bbox
        roi_mask[y:y+h, x:x+w] = 255
        
        # Keep only pixels inside ROI
        return cv2.bitwise_and(mask, roi_mask)
    
    def _temporal_validation(self, led_index, cx, cy):
        """Validate detection is stable across frames."""
        history = self.detection_history[led_index]
        history.append((cx, cy))
        
        if len(history) < self.REQUIRED_CONSECUTIVE:
            return False, f"Need {self.REQUIRED_CONSECUTIVE} frames (have {len(history)})"
        
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
        """Calculate quality score 0-1."""
        # Area score - prefer 150-500 pixels
        if 150 <= area <= 500:
            area_score = 1.0
        elif area < 150:
            area_score = area / 150
        else:
            area_score = max(0.2, 1.0 - (area - 500) / 1000)
        
        circ_score = circ
        brightness_score = min(1.0, (brightness - 200) / 50.0)
        brightness_score = max(0, brightness_score)
        change_score = 1.0 if is_new_change else 0.4
        
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
        """Main vision loop with ROI constraint."""
        import sys
        print("="*70, file=sys.stderr, flush=True)
        print("🎬 VISION WITH ROI TRACKING", file=sys.stderr, flush=True)
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
        
        # Warm up
        for i in range(10):
            cap.read()
        
        print("✅ Camera ready", file=sys.stderr, flush=True)
        
        # Get frame dimensions
        ret, test_frame = cap.read()
        frame_height, frame_width = test_frame.shape[:2]
        print(f"📐 Frame: {frame_width}x{frame_height}", file=sys.stderr, flush=True)
        
        # CSV setup
        if self.record:
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            csv_filename = f"vision_blobs_{ts}.csv"
            self.csv_file = open(csv_filename, "w", newline="")
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow([
                "frame_ts", "led_index", "cx_px", "cy_px",
                "area", "circ", "quality", "brightness", 
                "is_new_change", "temporal_valid", "accepted",
                "roi_active"  # NEW
            ])
            print(f"💾 CSV: {csv_filename}")
        
        # Start LED cycling
        if not self.calibration_mode:
            await self.client.write_gatt_char(LED_WRITE_UUID, CMD_START, response=False)
            print("✅ LED cycling started", file=sys.stderr, flush=True)
        
        prev_time = time.time()
        
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
                
                # === ROI UPDATE ===
                self.roi_update_counter += 1
                if self.roi_update_counter >= self.roi_update_interval:
                    self.roi_update_counter = 0
                    self.roi_bbox = self._update_roi_from_fingers(frame_width, frame_height)
                
                # Track if using ROI this frame
                using_roi = self.roi_enabled and self.roi_bbox is not None
                if using_roi:
                    self.stats['roi_active'] += 1
                else:
                    self.stats['roi_full_frame'] += 1
                
                # Convert to grayscale
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                blurred = cv2.GaussianBlur(gray, (5, 5), 0)
                
                # === BRIGHTNESS CHANGE DETECTION ===
                _, current_bright = cv2.threshold(blurred, self.BRIGHTNESS_THRESH, 255, cv2.THRESH_BINARY)
                
                if self.prev_bright_mask is not None and not self.calibration_mode:
                    new_bright = cv2.bitwise_and(
                        current_bright,
                        cv2.bitwise_not(self.prev_bright_mask)
                    )
                else:
                    new_bright = current_bright.copy()
                
                self.prev_bright_mask = current_bright.copy()
                
                # === APPLY ROI CONSTRAINT ===
                # This is where the magic happens - constrain search to hand area!
                new_bright = self._apply_roi_mask(new_bright)
                current_bright_clean = self._apply_roi_mask(current_bright)
                
                # Clean up masks
                new_bright = cv2.morphologyEx(new_bright, cv2.MORPH_OPEN, self.kernel)
                new_bright = cv2.morphologyEx(new_bright, cv2.MORPH_CLOSE, self.kernel, iterations=2)
                
                current_bright_clean = cv2.morphologyEx(current_bright_clean, cv2.MORPH_OPEN, self.kernel)
                current_bright_clean = cv2.morphologyEx(current_bright_clean, cv2.MORPH_CLOSE, self.kernel, iterations=2)
                
                # Find contours (now only in ROI!)
                new_contours, _ = cv2.findContours(new_bright, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                all_bright_contours, _ = cv2.findContours(current_bright_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
                # Process candidates (same as before)
                candidates = []
                
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
                    
                    is_duplicate = any(
                        abs(cand["center"][0] - cx) < 20 and 
                        abs(cand["center"][1] - cy) < 20 
                        for cand in candidates
                    )
                    if is_duplicate:
                        continue
                    
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
                
                # === SELECT BEST BLOB ===
                accepted = None
                reason = "no_candidates"
                
                if candidates:
                    candidates.sort(key=lambda b: b['quality'], reverse=True)
                    best = candidates[0]
                    cx, cy = best["center"]
                    
                    if best["quality"] < 0.5:
                        reason = f"Q={best['quality']:.2f}"
                        self.stats['rejected_quality'] += 1
                    else:
                        temporal_valid, temporal_reason = self._temporal_validation(self.current_led, cx, cy)
                        
                        if temporal_valid:
                            accepted = best
                            self.stats['accepted'] += 1
                            self.roi_no_detection_counter = 0  # Reset
                        else:
                            reason = temporal_reason
                            self.stats['rejected_temporal'] += 1
                else:
                    self.stats['no_blobs'] += 1
                    self.roi_no_detection_counter += 1
                    
                    # Auto-reset ROI if too many consecutive failures
                    if self.roi_no_detection_counter > self.roi_auto_reset_threshold:
                        print("⚠️  Too many failures - resetting ROI")
                        self.roi_bbox = None
                        self.roi_no_detection_counter = 0
                
                # === VISUALIZATION ===
                display = frame.copy()
                
                # Draw ROI box
                if using_roi:
                    x, y, w, h = self.roi_bbox
                    cv2.rectangle(display, (x, y), (x+w, y+h), 
                                 (255, 255, 0), 3)  # YELLOW box
                    cv2.putText(display, "ROI", (x+5, y+25),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
                
                # Draw candidates
                for cand in candidates:
                    cx, cy = cand["center"]
                    x, y, w, h = cv2.boundingRect(cand["contour"])
                    
                    if cand == accepted:
                        color = (0, 255, 0)
                        thickness = 3
                        label_color = (0, 255, 0)
                    elif cand["is_new_change"]:
                        color = (255, 165, 0)
                        thickness = 2
                        label_color = (255, 165, 0)
                    else:
                        color = (128, 128, 128)
                        thickness = 1
                        label_color = (128, 128, 128)
                    
                    cv2.rectangle(display, (x, y), (x + w, y + h), color, thickness)
                    cv2.circle(display, (cx, cy), 5, color, -1)
                    
                    change_marker = "★" if cand["is_new_change"] else "○"
                    info = f"{change_marker} Q:{cand['quality']:.2f}"
                    cv2.putText(display, info, (cx + 8, cy - 8),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, label_color, 1)
                
                # Rejection reason
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
                        accepted['is_new_change'], True, True, using_roi
                    ])
                
                # === OVERLAYS ===
                now = time.time()
                fps = 1/(now - prev_time) if (now - prev_time) > 0 else 0
                prev_time = now
                
                # LED indicator
                led_color = (0, 255, 0) if blob_centers else (0, 0, 255)
                cv2.putText(display, f"LED:{self.current_led}", (10, 40),
                           cv2.FONT_HERSHEY_SIMPLEX, 1.2, led_color, 3)
                
                # ROI status
                roi_text = "ROI:ON" if using_roi else "ROI:OFF"
                roi_color = (255, 255, 0) if using_roi else (128, 128, 128)
                cv2.putText(display, roi_text, (10, 90),
                           cv2.FONT_HERSHEY_SIMPLEX, 1.0, roi_color, 2)
                
                # FPS
                cv2.putText(display, f"FPS:{fps:.1f}", (10, 140),
                           cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 0), 2)
                
                # Acceptance rate
                if self.stats['total'] > 10:
                    rate = 100 * self.stats['accepted'] / self.stats['total']
                    rate_color = (0, 255, 0) if rate > 70 else (255, 165, 0) if rate > 40 else (0, 0, 255)
                    cv2.putText(display, f"Accept:{rate:.0f}%", (10, 190),
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
                
                # === DISPLAY ===
                cv2.imshow("ASL Vision", display)
                
                # Keyboard controls
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('o'):  # Toggle ROI
                    self.roi_enabled = not self.roi_enabled
                    print(f"🔄 ROI: {'ENABLED' if self.roi_enabled else 'DISABLED'}")
                elif key == ord('r'):  # Reset ROI
                    self.roi_bbox = None
                    print("🔄 ROI reset")
                elif key == ord(']'):
                    self.BRIGHTNESS_THRESH += 10
                    print(f"📈 Brightness: {self.BRIGHTNESS_THRESH}")
                elif key == ord('['):
                    self.BRIGHTNESS_THRESH = max(150, self.BRIGHTNESS_THRESH - 10)
                    print(f"📉 Brightness: {self.BRIGHTNESS_THRESH}")
                elif key == ord('}'):  # Increase ROI margin
                    self.roi_margin += 50
                    print(f"📈 ROI margin: {self.roi_margin}px")
                elif key == ord('{'):  # Decrease ROI margin
                    self.roi_margin = max(100, self.roi_margin - 50)
                    print(f"📉 ROI margin: {self.roi_margin}px")
                
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
            print(f"  ROI active: {self.stats['roi_active']} ({100*self.stats['roi_active']/max(1,self.stats['total']):.1f}%)")
            print(f"  Full frame: {self.stats['roi_full_frame']}")
            print(f"  Accepted: {self.stats['accepted']}")
            print(f"  Rejected (quality): {self.stats['rejected_quality']}")
            print(f"  Rejected (temporal): {self.stats['rejected_temporal']}")
            print(f"  No blobs: {self.stats['no_blobs']}")
            
            if self.stats['total'] > 0:
                rate = 100 * self.stats['accepted'] / self.stats['total']
                print(f"  Acceptance rate: {rate:.1f}%")
            
            print("📴 Vision stopped")
    
    def _assign_fingers(self, blob_centers):
        """Assign detected blob to finger."""
        if not blob_centers or self.current_led < 0:
            return
        
        cv_finger = self.led_to_finger.get(self.current_led)
        if not cv_finger:
            return
        
        cx, cy = blob_centers[0]
        
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
        """Called by main.py."""
        self.finger_positions = mapping
    
    def get_packet(self):
        """Return latest CV packet."""
        return self.last_packet
