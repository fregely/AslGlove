"""
OpenCV blob detection synchronized with ESP32 LED flasher over BLE.
Tracks finger LEDs over time with simple smoothing and trajectories.
"""

import asyncio
from collections import deque
from typing import Dict, Tuple, List

from bleak import BleakScanner, BleakClient
from bleak.backends.characteristic import BleakGATTCharacteristic
import cv2
import numpy as np
import time

# pylint: disable=E1101
# mypy: ignore-errors

# -------------------------------------------------------------------
# BLE CONFIG
# -------------------------------------------------------------------

# --- BLE UUIDs (must match ESP32 NimBLE configuration) ---
SERVICE_UUID = "ffeeddcc-bbaa-0011-2233-445566778899"
LED_NOTIFY_UUID = "c4e7a180-7b2f-4c95-bfc5-1d5c62123456"   # notifications: LED index ready
LED_WRITE_UUID = "01234567-89ab-cdef-0123-456789abcdef"   # writes: START / NEXT

# Commands ESP32 expects
CMD_START = bytes([1])
CMD_NEXT = bytes([2])

# Global LED index from notifications
current_led: int = -1
notification_received: bool = False

# -------------------------------------------------------------------
# FINGER TRACKING STRUCTURES
# -------------------------------------------------------------------

# Finger IDs in some canonical order
FINGER_NAMES = ["thumb", "index", "middle", "ring", "pinky"]

# Mapping LED index -> finger name (for now hardcoded; later will be calibrated)
led_to_finger: Dict[int, str] = {
    0: "thumb",
    1: "index",
    2: "middle",
    3: "ring",
    4: "pinky",
}

class FingerTrack:
    """Tracks position and trajectory of a finger."""
    def __init__(self, name: str) -> None:
        self.name = name
        self.pos: Tuple[float, float] | None = None          # last raw position (x, y)
        self.smoothed_pos: Tuple[float, float] | None = None # EMA-smoothed position
        self.missing_frames: int = 0                         # how many frames we've lost it
        self.traj: deque[Tuple[float, float]] = deque(maxlen=50)  # recent trajectory (for J/Z (dynamic letters))


finger_tracks: Dict[str, FingerTrack] = {name: FingerTrack(name) for name in FINGER_NAMES}

# Smoothing + missing thresholds
EMA_ALPHA = 0.5             # 0 = super smooth, 1 = no smoothing
MAX_MISSING_FRAMES = 5      # not used aggressively yet, but tracked


# -------------------------------------------------------------------
# BLE NOTIFICATION HANDLER
# -------------------------------------------------------------------

def notification_handler(_sender: BleakGATTCharacteristic, data: bytearray) -> None:
    """
    Called anytime ESP32 sends a BLE notification.
    ESP32 sends 1 byte: the LED index (0–4).
    """
    global current_led, notification_received
    current_led = int(data[0])
    notification_received = True
    print(f"📡 BLE READY from ESP32 → LED {current_led}")


# -------------------------------------------------------------------
# FINGER TRACKING UPDATE
# -------------------------------------------------------------------

def update_finger_track_from_led(blob_centers: List[Tuple[int, int]]) -> None:
    """
    Use current_led + detected blob centers to update the corresponding finger track.
    - One LED is on at a time, so current_led tells us which finger we are seeing.
    - If multiple blobs exist, choose the one closest to previous smoothed position.
    - If none exist, mark that finger as missing and keep last-known position.
    """
    global current_led

    # If we don't know how to map this LED, bail
    if current_led not in led_to_finger:
        return

    finger_name = led_to_finger[current_led]
    track = finger_tracks[finger_name]

    # No blobs: mark missing, but don't kill smoothed_pos
    if not blob_centers:
        track.missing_frames += 1
        # Optional: if track.missing_frames > MAX_MISSING_FRAMES, we could switch to IMU-only
        return

    # Choose which blob to use:
    # If we already have a smoothed position, pick nearest blob.
    if track.smoothed_pos is not None:
        px, py = track.smoothed_pos
        cx, cy = min(
            blob_centers,
            key=lambda p: (p[0] - px) ** 2 + (p[1] - py) ** 2,
        )
    else:
        # No previous position → take the first one
        cx, cy = blob_centers[0]

    # Reset missing counter; we have a valid detection
    track.missing_frames = 0

    # Exponential moving average smoothing
    if track.smoothed_pos is None:
        sm_x, sm_y = float(cx), float(cy)
    else:
        sx, sy = track.smoothed_pos
        sm_x = EMA_ALPHA * cx + (1.0 - EMA_ALPHA) * sx      # should figure out what this is actually doing
        sm_y = EMA_ALPHA * cy + (1.0 - EMA_ALPHA) * sy      # should figure out what this is actually doing

    # Update track
    track.pos = (float(cx), float(cy))
    track.smoothed_pos = (sm_x, sm_y)
    track.traj.append(track.smoothed_pos)


def draw_finger_tracks(frame: np.ndarray) -> None:
    """
    Draw smoothed finger positions and trajectories onto the frame.
    """
    for name, track in finger_tracks.items():
        # Draw smoothed position (if we have one)
        if track.smoothed_pos is not None:
            x, y = map(int, track.smoothed_pos)
            # Circle at smoothed point
            cv2.circle(frame, (x, y), 6, (255, 0, 255), 2)
            # Label with first letter of finger
            cv2.putText(
                frame,
                name[0].upper(),
                (x + 5, y - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 0, 255),
                2,
            )

        # Draw trajectory for that finger
        if len(track.traj) > 1:
            pts = np.array(track.traj, dtype=np.int32).reshape((-1, 1, 2))
            cv2.polylines(frame, [pts], isClosed=False, color=(0, 0, 255), thickness=2)


# -------------------------------------------------------------------
# MAIN ASYNC LOOP
# -------------------------------------------------------------------

async def run_ble_opencv() -> None:
    """Main async loop to synchronize BLE LED flasher with OpenCV blob detection."""
    global notification_received, current_led

    print("🔍 Scanning for ESP32...")
    device = await BleakScanner.find_device_by_filter(
        lambda d, _: (
            d.name is not None
            and ("ASL_Glove" in d.name or "NIMBLE" in d.name.upper())
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
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))  # type: ignore[attr-defined]
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        cap.set(cv2.CAP_PROP_FPS, 60)

        if not cap.isOpened():
            print("❌ Could not open camera.")
            return

        print("📸 Camera ready.")

        # Thresholding / blob detection parameters
        THRESH = 240
        kernel = np.ones((3, 3), np.uint8)
        MIN_AREA = 300
        MAX_AREA = 10000
        MIN_CIRC = 0.6

        prev = time.time()
        frame_count = 0

        while True:
            # ---------- WAIT FOR ESP32 READY ----------
            # Wait until ESP32 tells us which LED is on
            while not notification_received:
                await asyncio.sleep(0.001)  # 1 ms

            # Clear flag for next iteration
            notification_received = False

            # ---------- CAPTURE FRAME ----------
            ret, frame = cap.read()
            if not ret:
                print("❌ Camera frame error.")
                break

            frame_count += 1

            # ---------- PROCESS IMAGE ----------
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)

            _, mask = cv2.threshold(blurred, THRESH, 255, cv2.THRESH_BINARY)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            blob_centers: List[Tuple[int, int]] = []
            # --------- * this is where IMU data could be integrated * ---------

            for c in contours:
                area = cv2.contourArea(c)
                if area < MIN_AREA or area > MAX_AREA:
                    continue

                per = cv2.arcLength(c, True)
                circ = 4 * np.pi * area / (per * per + 1e-9)
                if circ < MIN_CIRC:
                    continue

                M = cv2.moments(c)
                if M["m00"] == 0:
                    continue

                cx, cy = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
                blob_centers.append((cx, cy))

                x, y, w, h = cv2.boundingRect(c)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # ---------- UPDATE FINGER TRACKS ----------
            update_finger_track_from_led(blob_centers)

            # ---------- FPS ----------
            now = time.time()
            fps = 1.0 / (now - prev)
            prev = now

            cv2.putText(
                frame,
                f"LED:{current_led}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (255, 255, 0),
                2,
            )
            cv2.putText(
                frame,
                f"FPS:{fps:.1f}",
                (10, 70),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 255),
                2,
            )

            # Draw smoothed finger positions + trajectories
            draw_finger_tracks(frame)

            cv2.imshow("detections", frame)

            # ---------- SEND NEXT TO ESP32 ----------
            await client.write_gatt_char(LED_WRITE_UUID, CMD_NEXT)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        # ---------- CLEANUP ----------
        await client.stop_notify(LED_NOTIFY_UUID)
        cap.release()
        cv2.destroyAllWindows()
        print("🔚 Closing BLE connection.")


# -------------------------------------------------------------------
# ENTRY POINT
# -------------------------------------------------------------------

if __name__ == "__main__":
    asyncio.run(run_ble_opencv())