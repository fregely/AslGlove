# asl/letter_recognizer2.py
# pylint: disable=E1101
# mypy: ignore-errors

import math
import statistics
import json
import asyncio

class LetterRecognizer2:
    """ ASL Letter Recognizer using fingertip positions and open-hand calibration."""

    def __init__(self):
        # Stores normalized distance baselines for open hand
        self.open_calibration = {}   # {("thumb","index"): avg_distance, ...}
        self.finger_names = ["thumb", "index", "middle", "ring", "pinky"]
        self.last_letter = None
        
        # Temporal filtering for stability
        self.letter_history = []  # Last N detected letters
        self.history_size = 12    # Number of frames to consider (~0.8s at 15fps)
        self.min_confidence = 9   # Minimum occurrences in history (9/12 = 75%)
        
        # Debouncing - prevent rapid letter changes
        self.frames_since_change = 0
        self.min_frames_between_changes = 5   # Require 5 stable frames before new letter (~0.33s at 15fps)
        
        # Hysteresis - prevent bouncing at threshold boundaries
        self.current_stable_letter = None  # Currently locked-in letter
        self.stable_letter_frames = 0      # How many frames we've been stable
        self.min_stable_frames = 8         # Frames needed to lock in a letter (~0.5s at 15fps)
        self.exit_multiplier = 1.4         # Require 40% more deviation to exit than to enter
        
        # Debug mode - prints raw detections before temporal filtering
        self.debug_mode = False
        self.debug_counter = 0
        self.verbose_debug = False  # Extra detailed geometry debug info

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
    # Helper: Angle between three points (in degrees)
    # -------------------------------
    def angle_between(self, p1, vertex, p2):
        """Calculate angle at vertex formed by p1-vertex-p2."""
        import numpy as np
        
        v1 = np.array(p1) - np.array(vertex)
        v2 = np.array(p2) - np.array(vertex)
        
        cos_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2) + 1e-8)
        cos_angle = np.clip(cos_angle, -1.0, 1.0)
        angle = np.arccos(cos_angle)
        return np.degrees(angle)
    
    # -------------------------------
    # Helper: Check if fingers are parallel
    # -------------------------------
    def are_parallel(self, f1, f2, threshold_deg=15):
        """Check if two fingers are roughly parallel (small angle between them)."""
        # This would need proper implementation with IMU orientation data
        # For now, use distance as proxy
        return self.dist(f1, f2) < 0.3
    
    # -------------------------------
    # Helper: Get finger spread (how far apart fingers are)
    # -------------------------------
    def finger_spread(self, packet):
        """Calculate average spread between adjacent fingers."""
        spreads = []
        finger_order = ["index", "middle", "ring", "pinky"]
        for i in range(len(finger_order) - 1):
            d = self.dist(packet[finger_order[i]], packet[finger_order[i+1]])
            spreads.append(d)
        return statistics.mean(spreads) if spreads else 0

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
        Enhanced with multiple distance metrics for better accuracy.

        Returns: { "index": "extended", "middle": "curled", ... }
        Returns None if calibration data is not available.
        """

        normalized = self.compute_normalized_distances(packet)
        
        # Return None if no calibration data
        if normalized is None:
            return None
            
        states = {}

        # Thresholds with hysteresis bands to reduce noise
        EXTENDED_MIN = 0.72      # Slightly relaxed for better detection
        CURL_MAX = 0.48          # Wider band for curl detection
        CLOSED_MAX = 0.28        # Adjusted threshold for closed state
        SEMI_MIN = 0.52          # Threshold between curl and semi

        for finger in ["index", "middle", "ring", "pinky"]:
            n = normalized[("thumb", finger)]

            if n > EXTENDED_MIN:
                states[finger] = "extended"
            elif n < CLOSED_MAX:
                states[finger] = "closed"
            elif n < CURL_MAX:
                states[finger] = "curled"
            elif n < SEMI_MIN:
                states[finger] = "semi"
            else:
                states[finger] = "extended"  # Treat ambiguous as extended

        # Thumb: Use multiple reference points for better accuracy
        thumb_to_index = normalized[("thumb", "index")]
        thumb_to_middle = normalized.get(("thumb", "middle"), thumb_to_index)
        
        # Average for more stable thumb detection
        avg_thumb = (thumb_to_index + thumb_to_middle) / 2
        
        if avg_thumb > EXTENDED_MIN:
            states["thumb"] = "extended"
        elif avg_thumb < CURL_MAX:
            states["thumb"] = "closed"
        else:
            states["thumb"] = "semi"

        return states
    
    
    # ------------------------------------------
    # Letter rules - REORGANIZED BY SPECIFICITY
    # Most specific/unique patterns first, general patterns last
    # ------------------------------------------
    def letter_from_states(self, states, packet):
        """
        Matches finger states and distances to ASL letters.
        Returns the detected letter or None.
        
        Enhanced with geometry analysis and confidence scoring.
        """
        
        # Get normalized distances for additional checks
        n = self.compute_normalized_distances(packet)
        
        # Return None if no calibration data
        if n is None:
            return None
        
        # Helper function to count extended fingers
        def count_extended():
            return sum(1 for f in ["index", "middle", "ring", "pinky"] if states[f] == "extended")
        
        def count_closed():
            return sum(1 for f in ["index", "middle", "ring", "pinky"] if states[f] in ["closed", "curled"])
        
        # Get finger spread for additional context
        avg_spread = self.finger_spread(packet)
        
        # ===========================================
        # PRIORITY 1: MOST UNIQUE PATTERNS
        # ===========================================
        
        # O: All fingers touch thumb making circle (very distinctive)
        # Check thumb-to-finger distances are all small AND fingers are close together
        thumb_distances = [n[("thumb", f)] for f in ["index", "middle", "ring", "pinky"]]
        avg_thumb_dist = statistics.mean(thumb_distances)
        max_thumb_dist = max(thumb_distances)
        
        # Hysteresis: Use different thresholds if already in 'O'
        if self.current_stable_letter == "O":
            # Staying in O - looser exit threshold
            if avg_thumb_dist < 0.35 and max_thumb_dist < 0.44:
                return "O"
        else:
            # Entering O - stricter entry threshold
            if avg_thumb_dist < 0.32 and max_thumb_dist < 0.40:
                return "O"
        
        # B: All 4 fingers extended and close together, thumb curled across palm
        if (states["index"] == "extended" and states["middle"] == "extended" and
            states["ring"] == "extended" and states["pinky"] == "extended" and
            states["thumb"] in ["curled", "closed", "semi"]):
            # Check fingers are relatively close together (not spread wide)
            if avg_spread < 0.45:  # Relaxed - fingers together
                return "B"
        
        # 4: All 4 fingers extended together, thumb tucked in (different from B)
        # This catches cases where all fingers extended but thumb position varies
        if (states["index"] == "extended" and states["middle"] == "extended" and
            states["ring"] == "extended" and states["pinky"] == "extended"):
            # If thumb is extended and away from fingers, might be "5" or general open hand
            # If thumb tucked (not extended far), treat as 4/B variant
            if states["thumb"] == "extended":
                # Check if thumb is far from fingers (open hand) or close (4)
                if n[("thumb", "index")] < 0.65:  # Thumb not far = 4
                    if avg_spread < 0.50:  # Fingers reasonably together
                        return "B"  # Treat as B (4 is not standard ASL letter)
        
        # I: Only pinky extended (others curled)
        if (states["pinky"] == "extended" and 
            states["index"] in ["curled", "closed"] and
            states["middle"] in ["curled", "closed"] and
            states["ring"] in ["curled", "closed"]):
            return "I"
        
        # Y: Thumb and pinky extended, others curled
        if (states["pinky"] == "extended" and states["thumb"] == "extended" and
            states["index"] in ["curled", "closed"] and
            states["middle"] in ["curled", "closed"] and
            states["ring"] in ["curled", "closed"]):
            return "Y"
        
        # ===========================================
        # PRIORITY 2: THREE-FINGER PATTERNS
        # ===========================================
        
        # W: Three fingers (index, middle, ring) extended, pinky curled
        if (states["index"] == "extended" and states["middle"] == "extended" and
            states["ring"] == "extended" and states["pinky"] in ["curled", "closed"]):
            # Ensure fingers are spread apart (wide spacing)
            index_middle_dist = n[("index", "middle")]
            middle_ring_dist = n[("middle", "ring")]
            
            # Hysteresis for W
            if self.current_stable_letter == "W":
                # Staying in W - looser exit
                if index_middle_dist > 0.25 and middle_ring_dist > 0.25:
                    return "W"
            else:
                # Entering W - stricter entry (fingers should be spread)
                if index_middle_dist > 0.30 and middle_ring_dist > 0.30:
                    return "W"
        
        # F: Index and thumb make circle, other 3 fingers extended
        if (states["middle"] == "extended" and states["ring"] == "extended" and
            states["pinky"] == "extended" and
            states["index"] in ["curled", "closed"]):
            # Hysteresis for F
            if self.current_stable_letter == "F":
                if n[("thumb", "index")] < 0.40:  # Looser exit (was 0.35)
                    return "F"
            else:
                if n[("thumb", "index")] < 0.35:  # Relaxed entry (was 0.30)
                    return "F"
        
        # ===========================================
        # PRIORITY 3: TWO-FINGER PATTERNS (CHECK SPACING)
        # ===========================================
        
        # Two-finger detection with better spacing thresholds + hysteresis
        if (states["index"] == "extended" and states["middle"] == "extended" and
            states["ring"] in ["curled", "closed"] and states["pinky"] in ["curled", "closed"]):
            
            im_dist = n[("index", "middle")]
            
            # Hysteresis for V/R/U to prevent bouncing
            if self.current_stable_letter == "V":
                # Staying in V - allow slightly closer before switching
                if im_dist > 0.45:
                    return "V"
            elif self.current_stable_letter == "R":
                # Staying in R - allow slightly wider before switching
                if im_dist < 0.35:
                    return "R"
            elif self.current_stable_letter == "U":
                # Staying in U - wider band
                if 0.25 <= im_dist <= 0.55:
                    return "U"
            else:
                # Entering - use strict thresholds
                # V: Index and middle extended APART (wide V shape)
                if im_dist > 0.50:  # Wide spacing
                    return "V"
                # R: Index and middle crossed/very close (fingers touching)
                elif im_dist < 0.30:  # Very close/crossed
                    return "R"
                # U: Index and middle extended together (parallel, medium spacing)
                elif 0.30 <= im_dist <= 0.50:  # Medium spacing
                    return "U"
        
        # K: Index and middle extended in V, thumb between them (very specific)
        if (states["index"] == "extended" and states["middle"] == "extended" and
            states["ring"] in ["curled", "closed"] and states["pinky"] in ["curled", "closed"]):
            ti = n[("thumb", "index")]
            tm = n[("thumb", "middle")]
            im = n[("index", "middle")]
            
            # Thumb should be roughly equidistant from both fingers
            if ti < 0.50 and tm < 0.50 and im > 0.35:
                thumb_symmetry = abs(ti - tm)
                if thumb_symmetry < 0.15:  # Thumb centered between fingers
                    return "K"
        
        # H: Index and middle extended sideways very close (parallel), others curled
        if (states["index"] == "extended" and states["middle"] == "extended" and
            states["ring"] in ["curled", "closed"] and states["pinky"] in ["curled", "closed"]):
            im = n[("index", "middle")]
            if im < 0.28:  # Very tight/parallel = H
                return "H"
        
        # P: Index pointing down, middle extended, thumb between them
        # (Similar to K but different angle)
        if (states["index"] == "extended" and states["middle"] == "extended" and
            states["ring"] in ["curled", "closed"] and states["pinky"] in ["curled", "closed"]):
            ti = n[("thumb", "index")]
            tm = n[("thumb", "middle")]
            if ti < 0.38 and tm > 0.42:
                # Thumb much closer to index than middle
                return "P"
        
        # ===========================================
        # PRIORITY 3B: THREE FINGERS EXTENDED (but not W pattern)
        # ===========================================
        
        # Three fingers extended (not matching W) - could be partial detection
        # This helps catch transitions or unclear hand positions
        if count_extended() == 3 and states["thumb"] == "extended":
            # Three fingers + thumb extended
            # Common: middle, ring, pinky extended (not W which has index)
            if (states["index"] in ["curled", "closed"] and 
                states["middle"] == "extended" and 
                states["ring"] == "extended" and 
                states["pinky"] == "extended"):
                # This is the shape from your log! Return F
                # (index closed, middle/ring/pinky extended, thumb extended)
                if n[("thumb", "index")] < 0.35:
                    return "F"  # Thumb touches closed index
        
        # ===========================================
        # PRIORITY 4: ONE FINGER + THUMB PATTERNS
        # ===========================================
        
        # L: Thumb and index extended at right angle, others curled
        if (states["thumb"] == "extended" and states["index"] == "extended" and
            states["middle"] in ["curled", "closed"] and
            states["ring"] in ["curled", "closed"] and
            states["pinky"] in ["curled", "closed"]):
            # L should have wide separation between thumb and index
            ti_dist = n[("thumb", "index")]
            # Hysteresis for L
            if self.current_stable_letter == "L":
                if ti_dist > 0.55:  # Looser exit
                    return "L"
            else:
                if ti_dist > 0.60:  # Strict entry (90 degree angle)
                    return "L"
        
        # D: Index extended, others curled, thumb touches middle finger
        if (states["index"] == "extended" and 
            states["middle"] in ["curled", "closed"] and
            states["ring"] in ["curled", "closed"] and 
            states["pinky"] in ["curled", "closed"]):
            if n[("thumb", "middle")] < 0.30:  # Thumb touching middle
                return "D"
        
        # G: Index extended sideways, thumb extended, others curled
        if (states["index"] == "extended" and 
            states["middle"] in ["curled", "closed"] and
            states["ring"] in ["curled", "closed"] and 
            states["pinky"] in ["curled", "closed"]):
            if states["thumb"] == "extended" and n[("thumb", "index")] > 0.55:
                return "G"
        
        # Q: Thumb and index pointing down
        if (states["thumb"] == "extended" and states["index"] == "extended" and
            states["middle"] in ["curled", "closed"] and 
            states["ring"] in ["curled", "closed"] and
            states["pinky"] in ["curled", "closed"]):
            if n[("thumb", "index")] < 0.45:
                return "Q"
        
        # ===========================================
        # PRIORITY 5: CLOSED FIST VARIATIONS
        # ===========================================
        
        # C: Curved hand shape (all fingers slightly curled in C shape)
        if all(states[f] in ["semi", "curled"] for f in ["index", "middle", "ring", "pinky"]):
            if 0.50 < n[("thumb", "index")] < 0.75:
                return "C"
        
        # Fist variations - check all four fingers are curled/closed
        if all(states[f] in ["curled", "closed"] for f in ["index", "middle", "ring", "pinky"]):
            ti = n[("thumb", "index")]
            tm = n.get(("thumb", "middle"), ti)
            tp = n.get(("thumb", "pinky"), ti)
            
            # E: Thumb very tightly across fingernails (smallest distance)
            if ti < 0.22:
                return "E"
            
            # S: Thumb across front of fingers (slightly looser than E)
            elif 0.22 <= ti < 0.32:
                return "S"
            
            # M: Thumb tucked under ring/pinky (closer to pinky)
            elif tp < 0.35 and ti < 0.45:
                return "M"
            
            # N: Thumb between index and middle (specific position)
            elif tm < 0.35 and ti > tm:
                return "N"
            
            # A: Thumb alongside fist (most loose)
            elif 0.32 <= ti < 0.50:
                return "A"
        
        # T: Thumb between index and middle (very specific)
        if all(states[f] in ["curled", "closed"] for f in ["index", "middle", "ring", "pinky"]):
            if n[("thumb", "index")] < 0.20 and 0.25 < n[("thumb", "middle")] < 0.40:
                return "T"
        
        # X: Index finger bent/hooked, others curled
        if (states["index"] == "semi" and 
            states["middle"] in ["curled", "closed"] and
            states["ring"] in ["curled", "closed"] and 
            states["pinky"] in ["curled", "closed"]):
            return "X"
        
        # Special case: 5 / Open hand (all fingers extended and spread)
        if count_extended() == 4 and states["thumb"] == "extended":
            # Check if fingers are spread wide
            if avg_spread > 0.50:
                # Wide open hand - not a letter
                pass  # Return None
            elif avg_spread < 0.45:
                # Fingers together - treat as B
                return "B"
        
        # ===========================================
        # FALLBACK: Ambiguous patterns
        # ===========================================
        
        # If we have 3 fingers extended (any combination), try to match something
        if count_extended() == 3:
            # Three fingers up - likely W, or partial hand shape
            # Check which fingers
            if states["pinky"] in ["curled", "closed"]:
                # Index, middle, ring extended - likely W
                return "W"
            elif states["index"] in ["curled", "closed"]:
                # Middle, ring, pinky extended - partial F or unclear
                # Check thumb distance
                if states["thumb"] == "extended" and n[("thumb", "index")] < 0.40:
                    return "F"
        
        # No match found
        return None

    # ------------------------------------------
    # Main entry with temporal filtering
    # (packet: fingertip 3D positions)
    # ------------------------------------------
    def recognize(self, packet, is_stationary=None, stationary_confidence=0.0):
        """
        Recognize ASL letter from finger positions.
        
        Args:
            packet: Finger positions dict {finger: (x,y,z)}
            is_stationary: Boolean indicating if hand is stationary (from ZUPT)
            stationary_confidence: Confidence score 0-1 for stationary state
        """
        states = self.classify_finger_states(packet)
        
        # Skip recognition if no calibration data
        if states is None:
            return None
        
        # Get raw detection for this frame
        raw_letter = self.letter_from_states(states, packet)
        
        # Debug mode: print raw detections
        if self.debug_mode and self.debug_counter < 100:
            self.debug_counter += 1
            print(f"[RAW {self.debug_counter}] Letter={raw_letter}, States={states}")
            
            # Verbose debug: show key distances
            if self.verbose_debug and raw_letter:
                n = self.compute_normalized_distances(packet)
                print(f"  Distances: T-I={n[('thumb','index')]:.2f}, I-M={n[('index','middle')]:.2f}, Spread={self.finger_spread(packet):.2f}")
        
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
            # No letters detected - reset stable state
            self.current_stable_letter = None
            self.stable_letter_frames = 0
            return None
            
        best_letter = max(letter_counts, key=letter_counts.get)
        best_count = letter_counts[best_letter]
        
        # Use fixed confidence threshold (no IMU/ZUPT dependency)
        required_confidence = self.min_confidence
        
        # Require minimum confidence
        if best_count < required_confidence:
            return None
        
        # Update stable letter tracking (hysteresis state machine)
        if best_letter == self.current_stable_letter:
            # Same letter - increment stability
            self.stable_letter_frames += 1
            final_letter = best_letter
        else:
            # Different letter detected
            if self.stable_letter_frames >= self.min_stable_frames:
                # We were stable on a previous letter - require strong evidence to switch
                if best_count >= required_confidence + 2:  # Extra requirement
                    # Strong enough to switch
                    self.current_stable_letter = best_letter
                    self.stable_letter_frames = 1
                    final_letter = best_letter
                else:
                    # Not strong enough - keep current stable letter
                    final_letter = self.current_stable_letter
            else:
                # Wasn't stable yet - accept new letter more easily
                self.current_stable_letter = best_letter
                self.stable_letter_frames = 1
                final_letter = best_letter
        
        # Apply debouncing - prevent rapid changes
        if final_letter == self.last_letter:
            # Same letter as before, don't output again
            self.frames_since_change += 1
            return None
        else:
            # Different letter detected
            # Allow first letter immediately, then require debouncing
            if self.last_letter is None:
                # First letter - output immediately
                self.last_letter = final_letter
                self.frames_since_change = 0
                return final_letter
            elif self.frames_since_change >= self.min_frames_between_changes:
                # Sufficient time has passed, allow change
                self.last_letter = final_letter
                self.frames_since_change = 0
                return final_letter
            else:
                # Too soon to change, keep waiting
                self.frames_since_change += 1
                return None