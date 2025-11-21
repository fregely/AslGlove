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

    Uses:
      - VisionProcessor.get_packet()
      - LED cycle controlled entirely by cv.py
      - BLE handled by ble_client.py

    For each LED in known_leds:
      - Wait until packet["led_index"] == LED
      - Wait until packet has at least one blob
      - Record (cx, cy)
    """

    def __init__(self, vision: object, known_leds: list[int]) -> None:
        self.vision = vision
        self.known_leds = known_leds
        self.result_map: dict[int, dict[str, any]] = {}

    async def _wait_for_blob(self, target_led: int, timeout: float = 5.0) -> tuple[int, int] | None:
        """Wait for cv.py to produce a blob for a specific LED."""
        start = time.time()

        while time.time() - start < timeout:
            packet = self.vision.get_packet()

            if not packet:
                await asyncio.sleep(0.01)
                continue

            led = packet.get("led_index", -1)
            blobs = packet.get("blob_centers", [])
            num = packet.get("num_blobs", 0)

            # Only proceed if cv.py is currently flashing the LED we need
            if led == target_led and num > 0:
                return blobs[0]   # First blob center

            await asyncio.sleep(0.01)

        return None

    async def run(self) -> None:
        print("\n🔧 === ASL Glove Calibration ===\n")
        print("Hold your hand steady while each LED flashes.\n")

        for led in self.known_leds:
            print(f"👉 Waiting for LED {led}...")

            blob = await self._wait_for_blob(led)

            if blob is None:
                print(f"⚠️ No blob detected for LED {led} (skipping)\n")
                continue

            cx, cy = blob
            finger = FINGER_NAMES.get(led, "unknown")

            print(f"   ✔ LED {led} detected → center = ({cx}, {cy})")

            self.result_map[led] = {
                "finger": finger,
                "center": [cx, cy],
            }

        # Save results
        with open("finger_map.json", "w") as f:
            json.dump(self.result_map, f, indent=4)

        print("\n💾 Saved LED→finger map to finger_map.json")
        print("🎉 Calibration finished!\n")