import json
import time
import asyncio

class Calibrator:
    """
    LED→finger calibration using firmware LED override mode.
    Python selects which LED is ON, ESP32 turns ONLY that LED on.
    Camera takes its centroid.
    """

    def __init__(self, vision: object, ble_client: object, led_gpio_order: list[int]) -> None:
        """ Initialize with VisionProcessor, BLEClient, and LED GPIO order. """
        self.vision = vision
        self.ble = ble_client
        self.led_gpio_order = led_gpio_order
        self.result_map = {}
        self.vision_task = None
        

    async def _wait_for_blob(self, timeout: float = 4.0) -> tuple | None:
        start = time.time()
        last_led = None
        blob_counts = []
        
        while time.time() - start < timeout:
            pkt = self.vision.get_packet()
            
            # Check if packet exists
            if not pkt:
                await asyncio.sleep(0.02)
                continue
                
            current_led = pkt.get("led_index", -1)
            blobs = pkt.get("blob_centers", [])
            
            # Track blob counts for debugging
            if current_led != last_led:
                blob_counts.append(len(blobs))
                last_led = current_led
            
            # We want exactly 1 blob for the current LED
            if len(blobs) == 1:
                print(f"   ✓ Found 1 blob at {blobs[0]}")
                return blobs[0]
            elif len(blobs) > 1:
                # Multiple blobs - take the brightest/largest one
                print(f"   ⚠ Found {len(blobs)} blobs, using first one")
                return blobs[0]
                
            await asyncio.sleep(0.02)
        
        print(f"   ⚠ Timeout: saw {blob_counts} blobs across {len(blob_counts)} frames")
        return None

    async def run(self) -> None:
        print("\n🔧 === LED Calibration Mode ===\n")

        finger_names = ["thumb", "index", "middle", "ring", "pinky"]
        
        self.vision_task = asyncio.create_task(self.vision.start())

        for idx, gpio in enumerate(self.led_gpio_order):
            finger = finger_names[idx]

            
            await asyncio.sleep(1.0)

            print(f"👉 Lighting LED for {finger} (GPIO={gpio})...")

            # Tell firmware to activate this LED only
            await self.ble.select_led(gpio)
            self.vision.current_led = gpio
            await asyncio.sleep(0.5)  # give LED/camera more time to settle and sync

            blob = await self._wait_for_blob()

            if blob is None:
                print(f"⚠️ No blob detected for {finger}")
                continue

            cx, cy = blob
            print(f"   ✔ {finger} centroid = ({cx},{cy})")

            self.result_map[str(gpio)] = {
                "finger": finger,
                "center": [cx, cy]
            }

        # Turn all LEDs OFF
        await self.ble.select_led(255)
        # Stop vision task
        if self.vision_task:
            self.vision_task.cancel()
            try:
                await self.vision_task
            except asyncio.CancelledError:
                pass


        with open("finger_map.json", "w") as f:
            json.dump(self.result_map, f, indent=4)

        print("\n💾 Saved to finger_map.json")
        
        # Reload the finger map into the vision processor
        self.vision.load_finger_map("finger_map.json")
        print("✅ Finger map loaded into vision processor")
        
        # Return the vision task so it can be stopped
        return self.vision_task