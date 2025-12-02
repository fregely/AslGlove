# asl/letter_recognizer2.py
# pylint: disable=E1101
# mypy: ignore-errors

import math
import statistics
import json
import asyncio

class LetterRecognizer2:

    def __init__(self):
        # Stores normalized distance baselines for open hand
        self.open_calibration = {}   # {("thumb","index"): avg_distance, ...}
        self.finger_names = ["thumb", "index", "middle", "ring", "pinky"]
        self.last_letter = None

    # -------------------------------
    # Helper: Euclidean distance
    # -------------------------------
    def dist(self, a, b):
        return math.sqrt(
            (a[0] - b[0])**2 +
            (a[1] - b[1])**2 +
            (a[2] - b[2])**2
        )

    # -------------------------------
    # Helper: list of all pair combinations
    # -------------------------------
    def _finger_pairs(self):
        pairs = []
        for i in range(len(self.finger_names)):
            for j in range(i + 1, len(self.finger_names)):
                pairs.append((self.finger_names[i], self.finger_names[j]))
        return pairs
    
    # ------------------------------------------
    # Store calibration table
    # Called externally after collecting open samples
    # ------------------------------------------
    def set_open_calibration(self, table):
        self.open_calibration = table
    
    # ------------------------------------------
    # Save calibration to JSON file
    # ------------------------------------------
    def save_calibration(self, filename="open_hand_calibration.json"):
        """Save open hand calibration data to JSON file."""
        # Convert tuple keys to strings for JSON serialization
        serializable = {f"{k[0]}-{k[1]}": v for k, v in self.open_calibration.items()}
        with open(filename, 'w') as f:
            json.dump(serializable, f, indent=4)
        print(f"💾 Saved open hand calibration to {filename}")
    
    # ------------------------------------------
    # Load calibration from JSON file
    # ------------------------------------------
    def load_calibration(self, filename="open_hand_calibration.json"):
        """Load open hand calibration data from JSON file."""
        try:
            with open(filename, 'r') as f:
                serializable = json.load(f)
            # Convert string keys back to tuples
            self.open_calibration = {tuple(k.split('-')): v for k, v in serializable.items()}
            print(f"✅ Loaded open hand calibration from {filename}")
            return True
        except FileNotFoundError:
            print(f"⚠️ Calibration file {filename} not found")
            return False
        except Exception as e:
            print(f"❌ Error loading calibration: {e}")
            return False
    
    # -------------------------------
    # 1. Calibration: open hand
    # -------------------------------
    async def calibrate_open_hand(self, vision_processor, samples=50, save=True):
        """
        Collects fingertip positions while the user holds an open hand
        and computes average distances between all fingertip pairs.
        
        Args:
            vision_processor: VisionProcessor instance with finger_positions
            samples: Number of samples to collect
            save: Whether to save calibration to file
        """

        print("\n--- OPEN HAND CALIBRATION ---")
        print("Please hold your hand completely open and still.")
        print(f"Collecting {samples} samples...")
        
        # Temporary storage for distances
        collected = {pair: [] for pair in self._finger_pairs()}

        count = 0
        last_reported = 0
        try:
            while count < samples:
                # Get finger positions from vision processor
                if not vision_processor.finger_positions:
                    await asyncio.sleep(0.05)
                    continue
                    
                packet = vision_processor.finger_positions
                
                # Debug: Show what we have
                available_fingers = list(packet.keys())
                if len(available_fingers) != last_reported:
                    print(f"  Available fingers: {available_fingers}")
                    last_reported = len(available_fingers)
                
                # Verify all fingers are present
                if not all(f in packet for f in self.finger_names):
                    await asyncio.sleep(0.05)
                    continue

                # Compute distances for this sample
                for (f1, f2) in self._finger_pairs():
                    d = self.dist(packet[f1], packet[f2])
                    collected[(f1, f2)].append(d)

                count += 1
                if count % 10 == 0:
                    print(f"  {count}/{samples} samples collected...")
                
                await asyncio.sleep(0.05)

            # Average distances
            for pair in collected:
                self.open_calibration[pair] = statistics.mean(collected[pair])

            print(f"\n✔ Successfully collected all {samples} samples!")
            print("✔ Open-hand calibration complete!\n")
            
            # Save to file
            if save:
                self.save_calibration()
                print("✅ Ready for letter recognition!\n")
                
        except (KeyboardInterrupt, asyncio.CancelledError):
            print("\n⚠️ Calibration cancelled by user")
            raise


    # -------------------------------
    # Helper: fetch fingertip positions
    # You will replace this with your real packet reading
    # -------------------------------
    async def _get_finger_positions(self, client):
        """
        Expected output format:
        {
            "thumb":  (x,y,z),
            "index":  (x,y,z),
            "middle": (x,y,z),
            "ring":   (x,y,z),
            "pinky":  (x,y,z)
        }
        """
        try:
            raw = await client.get_latest()  # <-- Replace with your BLE playback
            return raw["fingertips"]
        except:
            return None


    # -----------------------------------------------------
    # 2. Compute normalized distances for current packet
    # -----------------------------------------------------
    def compute_normalized_distances(self, packet):
        """
        Returns {pair: normalized_value}

        normalized = current_distance / open_calibration_distance
        """
        # Check if calibration data exists
        if not self.open_calibration:
            return None
            
        normalized = {}

        for (f1, f2) in self._finger_pairs():
            cur_d = self.dist(packet[f1], packet[f2])
            # Check if this specific pair exists in calibration
            if (f1, f2) not in self.open_calibration:
                return None
            base_d = self.open_calibration[(f1, f2)]
            normalized[(f1, f2)] = cur_d / base_d

        return normalized


    # -----------------------------------------------------
    # 3. Finger state classification (open / curled / closed)
    # -----------------------------------------------------
    def classify_finger_states(self, packet):
        """
        Uses thumb → finger normalized distances to determine
        if each finger is extended, curled, or closed.

        Returns: { "index": "extended", "middle": "curled", ... }
        Returns None if calibration data is not available.
        """

        normalized = self.compute_normalized_distances(packet)
        
        # Return None if no calibration data
        if normalized is None:
            return None
            
        states = {}

        # Thresholds — tune as needed!
        EXTENDED_MIN = 0.75
        CURL_MAX = 0.45
        CLOSED_MAX = 0.25  # finger touching thumb

        for finger in ["index", "middle", "ring", "pinky"]:

            n = normalized[("thumb", finger)]

            if n > EXTENDED_MIN:
                states[finger] = "extended"
            elif n < CLOSED_MAX:
                states[finger] = "closed"
            elif n < CURL_MAX:
                states[finger] = "curled"
            else:
                states[finger] = "semi"

        # Thumb status can be classified separately if you want
        # for now we do relative distance to index:
        ti = normalized[("thumb", "index")]
        if ti > EXTENDED_MIN:
            states["thumb"] = "extended"
        elif ti < CURL_MAX:
            states["thumb"] = "closed"
        else:
            states["thumb"] = "semi"

        return states
    
    
    # ------------------------------------------
    # Letter rules
    # ------------------------------------------
    def letter_from_states(self, states, packet):
        LETTER_PROFILES = {
            "A": {
                "index":  "curled",
                "middle": "curled",
                "ring":   "curled",
                "pinky":  "curled",
            },

            "B": {
                "index":  "extended",
                "middle": "extended",
                "ring":   "extended",
                "pinky":  "extended",
                "thumb":  "curled",
            },

            "L": {
                "thumb": "extended",
                "index": "extended",
                "middle": "curled",
                "ring": "curled",
                "pinky":"curled",
            },

            "I": {
                "pinky": "extended",
                "index": "curled",
                "middle":"curled",
                "ring":  "curled",
            },

            "Y": {
                "pinky": "extended",
                "thumb": "extended",
                "index": "curled",
                "middle": "curled",
                "ring": "curled",
            },
        }

        # exact profile match
        for letter, expected in LETTER_PROFILES.items():
            if all(states.get(f) == expected[f] for f in expected):
                return letter

        # need normalized distances too
        n = self.compute_normalized_distances(packet)

        # C shape
        if (
            0.45 < n[("thumb","pinky")] < 0.85 and
            0.45 < n[("index","ring")] < 0.85
        ):
            return "C"

        # D shape
        if (
            states["index"] == "extended" and
            states["middle"] == "curled" and
            states["ring"] == "curled" and
            states["pinky"] == "curled"
        ):
            return "D"

        # O shape
        if all(
            n[("thumb",f)] < 0.35
            for f in ["index","middle","ring","pinky"]
        ):
            return "O"

        # R vs U:
        if (
            states["index"] == "extended" and
            states["middle"] == "extended" and
            states["ring"] == "curled" and
            states["pinky"] == "curled"
        ):
            if n[("index","middle")] < 0.55:
                return "R"
            else:
                return "U"

        # V vs K
        if (
            states["index"] == "extended" and
            states["middle"] == "extended" and
            states["ring"] == "curled" and
            states["pinky"] == "curled"
        ):
            if (
                n[("thumb","index")] < n[("index","middle")] and
                n[("thumb","middle")] < n[("index","middle")]
            ):
                return "K"
            else:
                return "V"

        return None

    # ------------------------------------------
    # Main entry
    # (packet: fingertip 3D positions)
    # ------------------------------------------
    def recognize(self, packet):
        states = self.classify_finger_states(packet)
        
        # Skip recognition if no calibration data
        if states is None:
            return None
            
        letter = self.letter_from_states(states, packet)

        if letter and letter != self.last_letter:
            self.last_letter = letter
            return letter
        return None