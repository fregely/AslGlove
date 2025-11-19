# calibration.py
import json
import time
import asyncio

FINGER_NAMES = {
    0: "thumb",
    1: "index",
    2: "middle",
    3: "ring",
    4: "pinky",
}

class Calibrator:
    """
    Non-interactive LED calibration.

    Assumes:
      - VisionProcessor is already running and cycling LEDs via BLE.
      - VisionProcessor exposes get_packet(), returning dicts like:
        {
            "timestamp": float,
            "led_index": int,
            "num_blobs": int,
            "blob_centers": [(cx, cy), ...],
        }

    For each LED in known_leds:
      - Waits until VisionProcessor reports a blob for that led_index
      - Records the first blob center
    Saves:
      {
        "<led>": { "center": [cx, cy] },
        ...
      }
    """
    def __init__(self, vision: object, client: object, known_leds: list[int]) -> None:
        """ Initialize with VisionProcessor, BLE client, and known LED indices. """
        self.vision = vision
        self.client = client
        self.known_leds = known_leds
        self.result_map: dict[int, dict[str, object]] = {}
        # led_index -> {"finger": ..., "center": (cx, cy)}

    async def _wait_for_blob(self, target_led: int, timeout: float = 5.0) -> tuple[int, int] | None:
        """Wait for a stable blob from VisionProcessor for a specific LED."""
        start = time.time()

        while time.time() - start < timeout:
            
            packet = self.vision.get_packet()
            print("DEBUG PACKET RECEIVED:", packet)

            if packet is None:
                await asyncio.sleep(0.01)
                continue

            if packet["led_index"] != target_led:
                await asyncio.sleep(0.01)
                continue

            if packet.get("num_blobs", 0) >= 1:
                centers = packet.get("blob_centers", [])
                if centers:
                    return centers[0]

            await asyncio.sleep(0.01)

        return None

    async def run(self) -> None:
        print("\n🔧 === ASL Glove Calibration ===\n")
        print("Place your hand steady and hold each finger up when its LED lights.")
        print("We will step through the LEDs one-by-one.\n")

        for led in self.known_leds:
            print(f"👉 Waiting for blob from LED {led}...")
            
            # IMPORTANT: we do NOT write to BLE here.
            # VisionProcessor is already stepping LEDs (CMD_NEXT over BLE).
            # We just wait for the correct led_index to appear in packets.
            
            blob = await self._wait_for_blob(led)

            if blob is None:
                print(f"⚠️ Could not detect blob for LED {led} within timeout. Skipping.\n")
                continue

            cx, cy = blob
            finger_name = FINGER_NAMES.get(led, "unknown")
            
            print(f"   ✔ LED {led} → center = ({cx}, {cy})")

            self.result_map[led] = {
                "finger": finger_name,
                "center": [cx, cy],
            }

            print(f"   → Mapped LED {led}\n")

        # Save map
        with open("finger_map.json", "w") as f:
            json.dump(self.result_map, f, indent=4)

        print("\n💾 Saved calibration to finger_map.json")
        print("🎯 Calibration complete!\n")