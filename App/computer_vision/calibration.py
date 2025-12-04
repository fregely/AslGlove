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
        best_blob = None
        best_blob_score = -1  # Track brightest blob across frames
        
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
            
            # Collect best blob across multiple frames
            if len(blobs) == 1:
                # For single blob, use it if we see it consistently
                if best_blob is None or best_blob == blobs[0]:
                    best_blob = blobs[0]
                    best_blob_score += 1
                    
                # If we've seen the same blob 3+ times, that's our LED
                if best_blob_score >= 3:
                    print(f"   ✓ Found stable blob at {best_blob}")
                    return best_blob
                    
            elif len(blobs) > 1:
                # Multiple blobs - this might be noise or reflections
                print(f"   ⚠ Found {len(blobs)} blobs, waiting for cleaner signal...")
                
            await asyncio.sleep(0.02)
        
        # Timeout - return best blob if we found any
        if best_blob is not None:
            print(f"   ⚠ Timeout but found blob at {best_blob} (seen {best_blob_score} times)")
            return best_blob
            
        print(f"   ❌ Timeout: saw {blob_counts} blobs across {len(blob_counts)} frames")
        return None

    def _validate_calibration(self) -> bool:
        """
        Validate that fingers are spread apart enough for accurate recognition.
        Returns True if calibration is good, False if fingers too close.
        """
        MIN_DISTANCE = 100  # Minimum pixels between adjacent fingers
        
        positions = []
        finger_order = ["thumb", "index", "middle", "ring", "pinky"]
        
        # Extract positions in finger order
        for gpio_str, data in self.result_map.items():
            finger = data["finger"]
            center = data["center"]
            positions.append((finger, center))
        
        # Sort by finger order
        positions.sort(key=lambda x: finger_order.index(x[0]))
        
        print("\n📏 Calibration Validation:")
        print("=" * 60)
        
        all_good = True
        for i in range(len(positions)):
            finger1, (x1, y1) = positions[i]
            print(f"  {finger1:6s}: ({x1:4d}, {y1:4d})", end="")
            
            if i < len(positions) - 1:
                finger2, (x2, y2) = positions[i + 1]
                distance = ((x2 - x1)**2 + (y2 - y1)**2)**0.5
                
                if distance < MIN_DISTANCE:
                    print(f"  ❌ {distance:5.1f}px to {finger2} (TOO CLOSE!)")
                    all_good = False
                else:
                    print(f"  ✓ {distance:5.1f}px to {finger2}")
            else:
                print()  # Last finger, just newline
        
        print("=" * 60)
        
        if not all_good:
            print("⚠️  CALIBRATION FAILED: Fingers too close together!")
            print("💡 Please spread your fingers WIDE apart and try again.")
            print("   Aim for at least 100 pixels between each finger.")
            return False
        else:
            print("✅ Calibration looks good - fingers well separated!")
            return True

    async def run(self) -> None:
        print("\n🔧 === LED Calibration Mode ===\n")
        print("📌 IMPORTANT: Spread your fingers WIDE apart during calibration!")
        print("   - Make a 'star' shape with your hand")
        print("   - Keep fingers at least 100 pixels apart")
        print("   - Hold steady when each LED lights up\n")

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

        # Validate calibration before saving
        if not self._validate_calibration():
            print("\n❌ Calibration rejected - please try again with fingers spread wider!")
            return self.vision_task

        with open("finger_map.json", "w") as f:
            json.dump(self.result_map, f, indent=4)

        print("\n💾 Saved to finger_map.json")
        
        # Reload the finger map into the vision processor
        self.vision.load_finger_map("finger_map.json")
        print("✅ Finger map loaded into vision processor")
        
        # Return the vision task so it can be stopped
        return self.vision_task