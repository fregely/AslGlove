"""
OpenCV blob detection synchronized with ESP32 LED flasher over BLE.
"""

import asyncio
from bleak import BleakScanner, BleakClient
from bleak.backends.characteristic import BleakGATTCharacteristic
import cv2
import numpy as np
import time

# pylint: disable=E1101
# mypy: ignore-errors

# --- BLE UUIDs (must match ESP32 NimBLE configuration) ---
SERVICE_UUID = "ffeeddcc-bbaa-0011-2233-445566778899"
LED_NOTIFY_UUID = "c4e7a180-7b2f-4c95-bfc5-1d5c62123456"
LED_WRITE_UUID = "01234567-89ab-cdef-0123-456789abcdef"

# Commands ESP32 expects
CMD_START = bytes([1])
CMD_NEXT  = bytes([2])

# Global LED index from notifications
current_led: int = -1
notification_received: bool = False

# ---------------- BLE NOTIFICATION HANDLER ----------------
def notification_handler(_sender: BleakGATTCharacteristic, data: bytearray) -> None:
    """
    Called anytime ESP32 sends a BLE notification.
    ESP32 sends 1 byte: the LED index (0–4).
    """
    global current_led, notification_received
    current_led = int(data[0])
    notification_received = True
    print(f"📡 BLE READY from ESP32 → LED {current_led}")

# ------------------- MAIN ASYNC LOOP ----------------------
async def run_ble_opencv() -> None:
    """Main async loop to synchronize BLE LED flasher with OpenCV blob detection."""
    
    print("🔍 Scanning for ESP32...")
    device = await BleakScanner.find_device_by_filter(
        lambda d, _: (
        d.name is not None and 
        ("ASL_Glove" in d.name or "NIMBLE" in d.name.upper())
    )
    )

    if device is None:
        print("❌ ESP32 not found.")
        return

    print(f"✅ Found ESP32: {device.address}")

    async with BleakClient(device) as client:
        print("🔗 Connected to ESP32 BLE.")

        # Subscribe to LED READY notifications
        await client.start_notify(LED_NOTIFY_UUID, notification_handler)

        # Tell ESP32 to start LED looping
        print("🚀 Sending START command...")
        await client.write_gatt_char(LED_WRITE_UUID, CMD_START)
        # --------- CAMERA SETUP ---------
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG')) # type: ignore[attr-defined]
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        cap.set(cv2.CAP_PROP_FPS, 60)

        if not cap.isOpened():
            print("❌ Could not open camera.")
            return

        print("📸 Camera ready.")

        THRESH = 240
        kernel = np.ones((3, 3), np.uint8)
        MIN_AREA = 300
        MAX_AREA = 10000
        MIN_CIRC = 0.6

        prev = time.time()
        frame_count = 0
        while True:
            global notification_received
            
            # ---------- WAIT FOR ESP32 READY ----------
            #notification_received = False
            # Wait until ESP32 sends READY
            while not notification_received:
                await asyncio.sleep(0.001)  # 1 ms

            notification_received = False
            # ---------- CAPTURE FRAME ----------
            ret, frame = cap.read()
            if not ret:
                print("❌ Camera frame error.")
                break

            frame_count += 1

            # ---------- PROCESS IMAGE ----------
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (5,5), 0)

            _, mask = cv2.threshold(blurred, THRESH, 255, cv2.THRESH_BINARY)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            blob_centers = []

            for c in contours:
                area = cv2.contourArea(c)
                if area < MIN_AREA or area > MAX_AREA:
                    continue
                per = cv2.arcLength(c, True)
                circ = 4 * np.pi * area / (per*per + 1e-9)
                if circ < MIN_CIRC:
                    continue

                M = cv2.moments(c)
                if M["m00"] == 0:
                    continue
                cx, cy = int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"])
                blob_centers.append((cx, cy))

                x,y,w,h = cv2.boundingRect(c)
                cv2.rectangle(frame, (x,y), (x+w,y+h), (0,255,0), 2)

            # ---------- FPS ----------
            now = time.time()
            fps = 1/(now-prev)
            prev = now

            cv2.putText(frame, f"LED:{current_led}", (10,30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,0), 2)
            cv2.putText(frame, f"FPS:{fps:.1f}", (10,70),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,255), 2)

            cv2.imshow("detections", frame)

            # ---------- SEND NEXT TO ESP32 ----------
            await client.write_gatt_char(LED_WRITE_UUID, CMD_NEXT)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # ---------- CLEANUP ----------
        await client.stop_notify(LED_NOTIFY_UUID)
        cap.release()
        cv2.destroyAllWindows()
        print("🔚 Closing BLE connection.")

# ---------------- ENTRY POINT ----------------
if __name__ == "__main__":
    asyncio.run(run_ble_opencv())