# pylint: disable=E1101
# mypy: ignore-errors

# ------------------------------
# VISION PROCESSING (OpenCV)
# ------------------------------
import time
import asyncio
import cv2
import numpy as np
import csv
from datetime import datetime

SERVICE_UUID = "ffeeddcc-bbaa-0011-2233-445566778899"
IMU_DATA_UUID = "c4e7a180-7b2f-4c95-bfc5-1d5c62123456"
LED_STATE_UUID = "01234567-89ab-cdef-0123-456789abcdef"  # FIXED - this was wrong!

LED_NOTIFY_UUID = LED_STATE_UUID
LED_WRITE_UUID = LED_STATE_UUID

CMD_START = bytes([1])
CMD_NEXT  = bytes([2])

class VisionProcessor:
    """ OpenCV vision processor synchronized with ESP32 LED flasher over BLE."""
    def __init__(self, client, record: bool = False, thresh: int = 240, min_area: int =300, max_area: int =10000, min_circ: float =0.6, pixel_per_mm: float =1.0):
        self.client = client
        self.current_led = -1
        self.ready_flag = False
        self.record = record # Enable/disable CSV logging
        
        # processing constants
        self.THRESH = thresh
        self.kernel = np.ones((3,3), np.uint8)
        self.MIN_AREA = min_area
        self.MAX_AREA = max_area
        self.MIN_CIRC = min_circ
        self.last_blob_centers = []
        self.last_timestamp = None
        self.last_packet = {}
        
        # LED → IMU offset (approx 7 mm above IMU)
        # PX_PER_MM should be calibrated; 1.0 is a reasonable starting point.
        self.LED_OFFSET_MM = 7.0
        self.PX_PER_MM = pixel_per_mm # TODO: measure mm→px scale and update this
        # Image coords: y increases downward, so to move LED "down" toward IMU,
        # we add a positive dy.
        self.LED_OFFSET_PX = (0, int(self.LED_OFFSET_MM * self.PX_PER_MM))
        
        # CSV logging
        self.csv_file = None
        self.csv_writer = None

    def handler(self, _ch, data: bytearray):
        """BLE callback: ESP32 says 'ready—capture next frame'"""
        if len(data) == 1:
            self.current_led = int(data[0])
            self.ready_flag = True

    async def start(self):
        """Start vision + LED sync."""
        print("📹 Starting OpenCV vision processor...")

        # Small delay to ensure BLE subscriptions are stable
        await asyncio.sleep(0.2)

        # Tell ESP32 to begin LED loop 
        await self.client.write_gatt_char(LED_WRITE_UUID, CMD_START)

        # Setup camera
        cap = cv2.VideoCapture(2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG')) # type: ignore[attr-defined]
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        cap.set(cv2.CAP_PROP_FPS, 60)

        if not cap.isOpened():
            print("❌ Could not open camera.")
            return
        
        # Setup CSV logging for blobs (only if recording is enabled)
        if self.record:
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"vision_blobs_{ts}.csv"
            self.csv_file = open(filename, "w", newline="", encoding="utf-8")
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(
                [
                    "frame_ts",
                    "led_index",
                    "blob_idx",
                    "cx_px",
                    "cy_px",
                    "area_px2",
                    "circularity",
                    "raw_cx_px",
                    "raw_cy_px",
                ]
            )
            print(f"💾 Logging blob data to {filename}")
        else:
            print("📝 Blob CSV logging disabled (run with --record to save blobs).")

        
        prev = time.time()

        while True:
            # Wait for ESP32 READY
            while not self.ready_flag:
                await asyncio.sleep(0.0005)
            self.ready_flag = False

            ret, frame = cap.read()
            if not ret:
                print("❌ Camera error")
                break

            # --- process frame ---
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (5,5), 0)

            _, mask = cv2.threshold(blurred, self.THRESH, 255, cv2.THRESH_BINARY)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel, iterations=2)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            blob_centers = []
            debug_info = []
            
            for c in contours:
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

                # Apply LED→IMU offset in image space
                dx, dy = self.LED_OFFSET_PX
                cx = int(raw_cx + dx)
                cy = int(raw_cy + dy)

                blob_centers.append((cx, cy))

                debug_info.append(
                    {
                        "center": (cx, cy),         # corrected center
                        "raw_center": (raw_cx, raw_cy),
                        "area": round(area),
                        "circ": round(circ, 2),
                    }
                )

                x, y, w, h = cv2.boundingRect(c)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                # Optional: draw corrected center marker
                cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)
                
                
            # --- LOGGING + CSV  ---
            frame_ts = time.time()

            # I got annoyed with all the console spam, so commenting out for now
            # if blob_centers:
            #     print(f"[VISION] LED {self.current_led}: {len(blob_centers)} blob(s) => {debug_info}")
            # else:
            #     print(f"[VISION] LED {self.current_led}: no blobs")

             # CSV: one row per blob; if none, optionally log a "no blob" row
            if self.csv_writer is not None:
                if blob_centers:
                    for idx, info in enumerate(debug_info):
                        cx, cy = info["center"]
                        raw_cx, raw_cy = info["raw_center"]
                        area = info["area"]
                        circ = info["circ"]
                        self.csv_writer.writerow(
                            [
                                frame_ts,
                                self.current_led,
                                idx,
                                cx,
                                cy,
                                area,
                                circ,
                                raw_cx,
                                raw_cy,
                            ]
                        )
                else:
                    # Optional: still log the LED flash with no blob
                    self.csv_writer.writerow(
                        [frame_ts, self.current_led, -1, "", "", "", "", "", ""]
                    )
                    
            self.last_blob_centers = blob_centers
            self.last_timestamp = frame_ts
                
            self.last_packet = {
                "cv_timestamp": frame_ts,
                "led_index": self.current_led,
                "blob_centers": blob_centers,
                "raw_blob_centers": [info["raw_center"] for info in debug_info],
                "num_blobs": len(blob_centers),
            }
            
            # ---------- FPS ----------
            now = time.time()
            fps = 1/(now-prev)
            prev = now

            cv2.putText(frame, f"LED:{self.current_led}", (10,30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,0), 2)
            cv2.putText(frame, f"FPS:{fps:.1f}", (10,70),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,255), 2)

            cv2.imshow("ASL Vision", frame)

            # Let ESP32 advance to next LED
            await self.client.write_gatt_char(LED_WRITE_UUID, CMD_NEXT)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()
        
        if self.csv_file is not None:
            self.csv_file.close()
            print("💾 Closed blob CSV file.")


        # await self.client.stop_notify(LED_NOTIFY_UUID)
        print("📴 Vision processor stopped.")
        
    def get_packet(self):
        """Return the most recent CV packet."""
        return self.last_packet
