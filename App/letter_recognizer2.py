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
        
        # Temporal filtering for stability
        self.letter_history = []  # Last N detected letters
        self.history_size = 5     # Number of frames to consider
        self.min_confidence = 3   # Minimum occurrences in history (3/5 = 60%)
        
        # Debouncing - prevent rapid letter changes
        self.frames_since_change = 0
        self.min_frames_between_changes = 3  # Require 3 stable frames before new letter

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

        # Thresholds with hysteresis bands to reduce noise
        EXTENDED_MIN = 0.70      # Lowered slightly for easier detection
        CURL_MAX = 0.50          # Increased to create wider bands
        CLOSED_MAX = 0.30        # Increased to reduce false closures

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
        """
        Matches finger states and distances to ASL letters.
        Returns the detected letter or None.
        """
        
        # Get normalized distances for additional checks
        n = self.compute_normalized_distances(packet)
        
        # Return None if no calibration data
        if n is None:
            return None
        
        # ===========================================
        # EXACT STATE MATCHES (simple letters)
        # ===========================================
        
        # A: All fingers curled into fist, thumb alongside
        if (states["index"] == "curled" and states["middle"] == "curled" and 
            states["ring"] == "curled" and states["pinky"] == "curled"):
            return "A"
        
        # B: All 4 fingers extended, thumb curled across palm
        if (states["index"] == "extended" and states["middle"] == "extended" and
            states["ring"] == "extended" and states["pinky"] == "extended" and
            states["thumb"] == "curled"):
            return "B"
        
        # I: Only pinky extended (others curled)
        if (states["pinky"] == "extended" and states["index"] == "curled" and
            states["middle"] == "curled" and states["ring"] == "curled"):
            return "I"
        
        # L: Thumb and index extended, others curled
        if (states["thumb"] == "extended" and states["index"] == "extended" and
            states["middle"] == "curled" and states["ring"] == "curled" and
            states["pinky"] == "curled"):
            return "L"
        
        # Y: Thumb and pinky extended, others curled
        if (states["pinky"] == "extended" and states["thumb"] == "extended" and
            states["index"] == "curled" and states["middle"] == "curled" and
            states["ring"] == "curled"):
            return "Y"
        
        # ===========================================
        # LETTERS REQUIRING DISTANCE CHECKS
        # ===========================================
        
        # C: Curved hand shape (all fingers slightly curled in C shape)
        if all(states[f] in ["semi", "curled"] for f in ["index", "middle", "ring", "pinky"]):
            if 0.55 < n[("thumb", "index")] < 0.80:
                return "C"
        
        # D: Index extended, others curled, thumb touches middle finger
        if (states["index"] == "extended" and states["middle"] == "curled" and
            states["ring"] == "curled" and states["pinky"] == "curled"):
            if n[("thumb", "middle")] < 0.3:  # Thumb touching middle
                return "D"
        
        # E: All fingers tightly curled, thumb across fingernails
        if all(states[f] == "curled" for f in ["index", "middle", "ring", "pinky"]):
            if n[("thumb", "index")] < 0.25:
                return "E"
        
        # F: Index and thumb make circle, other 3 fingers extended
        if (states["middle"] == "extended" and states["ring"] == "extended" and
            states["pinky"] == "extended" and
            states["index"] in ["curled", "closed", "semi"]):
            if n[("thumb", "index")] < 0.35:  # Thumb and index touching
                return "F"
        
        # G: Index and thumb extended sideways, others curled
        if (states["index"] == "extended" and states["middle"] == "curled" and
            states["ring"] == "curled" and states["pinky"] == "curled"):
            if n[("thumb", "index")] > 0.6:  # Thumb and index apart
                return "G"
        
        # H: Index and middle extended sideways, others curled
        if (states["index"] == "extended" and states["middle"] == "extended" and
            states["ring"] == "curled" and states["pinky"] == "curled"):
            if n[("index", "middle")] < 0.3:  # Fingers together
                return "H"
        
        # M: Thumb under 3 curled fingers
        if (states["index"] == "curled" and states["middle"] == "curled" and
            states["ring"] == "curled" and states["pinky"] == "curled"):
            if n[("thumb", "pinky")] < 0.35:
                return "M"
        
        # N: Thumb under 2 curled fingers (index and middle)
        if (states["index"] == "curled" and states["middle"] == "curled" and
            states["ring"] == "curled" and states["pinky"] == "curled"):
            if 0.35 < n[("thumb", "ring")] < 0.6:
                return "N"
        
        # O: All fingers touch thumb making circle
        if all(n[("thumb", f)] < 0.40 for f in ["index", "middle", "ring", "pinky"]):
            return "O"
        
        # P: Index pointing down, middle extended, thumb between them
        if (states["index"] == "extended" and states["middle"] == "extended" and
            states["ring"] == "curled" and states["pinky"] == "curled"):
            if n[("thumb", "index")] < 0.4:
                return "P"
        
        # Q: Thumb and index pointing down
        if (states["thumb"] == "extended" and states["index"] == "extended" and
            states["middle"] == "curled" and states["ring"] == "curled" and
            states["pinky"] == "curled"):
            if n[("thumb", "index")] < 0.4:
                return "Q"
        
        # R: Index and middle crossed, others curled
        if (states["index"] == "extended" and states["middle"] == "extended" and
            states["ring"] == "curled" and states["pinky"] == "curled"):
            if n[("index", "middle")] < 0.4:  # Fingers crossed/close
                return "R"
        
        # S: Fist with thumb across front
        if all(states[f] == "curled" for f in ["index", "middle", "ring", "pinky"]):
            if 0.3 < n[("thumb", "index")] < 0.5:
                return "S"
        
        # T: Thumb between index and middle
        if (states["index"] == "curled" and states["middle"] == "curled" and
            states["ring"] == "curled" and states["pinky"] == "curled"):
            if n[("thumb", "index")] < 0.25 and n[("thumb", "middle")] < 0.35:
                return "T"
        
        # U: Index and middle extended together, others curled
        if (states["index"] == "extended" and states["middle"] == "extended" and
            states["ring"] == "curled" and states["pinky"] == "curled"):
            if n[("index", "middle")] < 0.5:  # Fingers together
                return "U"
        
        # V: Index and middle extended apart, others curled
        if (states["index"] == "extended" and states["middle"] == "extended" and
            states["ring"] == "curled" and states["pinky"] == "curled"):
            if n[("index", "middle")] > 0.5:  # Fingers apart
                return "V"
        
        # W: Three fingers (index, middle, ring) extended, pinky curled
        if (states["index"] == "extended" and states["middle"] == "extended" and
            states["ring"] == "extended" and states["pinky"] in ["curled", "closed"]):
            # Additional check: ensure fingers are reasonably spread
            if n[("index", "ring")] > 0.40:
                return "W"
        
        # X: Index finger bent/hooked, others curled
        if (states["index"] == "semi" and states["middle"] == "curled" and
            states["ring"] == "curled" and states["pinky"] == "curled"):
            return "X"
        
        # K: Index and middle extended in V, thumb between them
        if (states["index"] == "extended" and states["middle"] == "extended" and
            states["ring"] == "curled" and states["pinky"] == "curled"):
            if (n[("thumb", "index")] < n[("index", "middle")] and
                n[("thumb", "middle")] < n[("index", "middle")]):
                return "K"
        
        # No match found
        return None

    # ------------------------------------------
    # Main entry with temporal filtering
    # (packet: fingertip 3D positions)
    # ------------------------------------------
    def recognize(self, packet):
        states = self.classify_finger_states(packet)
        
        # Skip recognition if no calibration data
        if states is None:
            return None
            
        # Get raw detection for this frame
        raw_letter = self.letter_from_states(states, packet)
        
        # Add to history (use None if no detection)
        self.letter_history.append(raw_letter)
        if len(self.letter_history) > self.history_size:
            self.letter_history.pop(0)
        
        # Need at least 2 frames for comparison
        if len(self.letter_history) < 2:
            return None
        
        # Count occurrences in history (excluding None)
        letter_counts = {}
        for l in self.letter_history:
            if l is not None:
                letter_counts[l] = letter_counts.get(l, 0) + 1
        
        # Find most common letter
        if not letter_counts:
            return None
            
        best_letter = max(letter_counts, key=letter_counts.get)
        best_count = letter_counts[best_letter]
        
        # Require minimum confidence (e.g., 3 out of 5 frames)
        if best_count < self.min_confidence:
            return None
        
        # Apply debouncing - prevent rapid changes
        if best_letter == self.last_letter:
            # Same letter as before, don't output again
            self.frames_since_change += 1
            return None
        else:
            # Different letter detected
            # Allow first letter immediately, then require debouncing
            if self.last_letter is None:
                # First letter - output immediately
                self.last_letter = best_letter
                self.frames_since_change = 0
                return best_letter
            elif self.frames_since_change >= self.min_frames_between_changes:
                # Sufficient time has passed, allow change
                self.last_letter = best_letter
                self.frames_since_change = 0
                return best_letter
            else:
                # Too soon to change, keep waiting
                self.frames_since_change += 1
                return None