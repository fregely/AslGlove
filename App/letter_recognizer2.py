# asl/letter_recognizer2.py
# pylint: disable=E1101
# mypy: ignore-errors

import math
import numpy as np

class LetterRecognizer2:
    """ASL Letter Recognizer using IMU curl angles + CV pixel distances."""

    def __init__(self):
        self.finger_names = ["thumb", "index", "middle", "ring", "pinky"]
        self.last_letter = None
        
        # Temporal filtering
        self.letter_history = []
        self.history_size = 5
        self.min_confidence = 3  # 3/5 frames = 60%
        
        # Debouncing
        self.frames_since_change = 0
        self.min_frames_between_changes = 3
        self.letter_output_once = False
        
        # Debug mode
        self.debug_mode = False

    # ===========================================
    # MAIN ENTRY POINT
    # ===========================================
    
    def classify(self, cv_positions, imu_orientations):
        """
        Main entry - uses RAW pixel positions + IMU angles.
        
        Args:
            cv_positions: {finger: (x, y)} in pixels
            imu_orientations: {finger: {'quaternion': [...], 'euler': [...]}}
        """
        if not cv_positions or not imu_orientations:
            return None
        
        # Compute IMU curl angles
        curl_angles = self.compute_curl_angles(imu_orientations)
        
        # Detect with temporal filtering
        return self.letter_from_features(cv_positions, curl_angles)
    
    # ===========================================
    # IMU CURL ANGLE COMPUTATION
    # ===========================================
    
    def compute_curl_angles(self, imu_orientations):
        """Convert quaternions to curl angles."""
        palm_quat = imu_orientations['thumb']['quaternion']
        
        angles = {}
        for finger in ['index', 'middle', 'ring', 'pinky']:
            finger_quat = imu_orientations[finger]['quaternion']
            angles[finger] = self._relative_pitch(palm_quat, finger_quat)
        
        return angles
    
    def _relative_pitch(self, q1, q2):
        """Get pitch angle between two quaternions."""
        _, pitch1, _ = self._quat_to_euler(q1)
        _, pitch2, _ = self._quat_to_euler(q2)
        return abs(pitch2 - pitch1)
    
    def _quat_to_euler(self, q):
        """Convert quaternion to Euler angles (degrees)."""
        w, x, y, z = q
        
        roll = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
        sinp = 2*(w*y - z*x)
        pitch = math.asin(np.clip(sinp, -1, 1))
        yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        
        return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))
    
    # ===========================================
    # HELPER FUNCTIONS
    # ===========================================
    
    def dist(self, a, b):
        """Euclidean distance between two points."""
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        dz = (a[2] - b[2]) if len(a) > 2 and len(b) > 2 else 0.0
        return math.sqrt(dx**2 + dy**2 + dz**2)
    
    # ===========================================
    # LETTER DETECTION WITH TEMPORAL FILTERING
    # ===========================================
    
    def letter_from_features(self, cv_positions, curl_angles):
        """
        Detect letter with temporal filtering.
        
        Args:
            cv_positions: Raw pixel coords {finger: (x,y)}
            curl_angles: IMU angles {finger: degrees}
        """
        if cv_positions is None or curl_angles is None:
            return None
        
        # Classify states from IMU angles
        states = {}
        EXTENDED_MAX = 30
        CURLED_MIN = 90
        
        for finger in ['index', 'middle', 'ring', 'pinky']:
            angle = curl_angles.get(finger, 0)
            if angle < EXTENDED_MAX:
                states[finger] = "extended"
            elif angle > CURLED_MIN:
                states[finger] = "closed"
            else:
                states[finger] = "semi"
        
        # Thumb state from CV pixel distance
        def px_dist(f1, f2):
            if f1 not in cv_positions or f2 not in cv_positions:
                return 9999
            return self.dist(cv_positions[f1], cv_positions[f2])
        
        ti_px = px_dist("thumb", "index")
        if ti_px > 200:
            states["thumb"] = "extended"
        elif ti_px < 100:
            states["thumb"] = "closed"
        else:
            states["thumb"] = "semi"
        
        print(f"  Computed states: {states}")
        print(f"  Thumb-index distance: {ti_px:.1f}px")

        # Get raw detection
        raw_detected = self.letter_from_states_enhanced(states, cv_positions, curl_angles)
        
        # ============================================
        # TEMPORAL FILTERING
        # ============================================
        
        # Add to history
        self.letter_history.append(raw_detected)
        if len(self.letter_history) > self.history_size:
            self.letter_history.pop(0)
        
        # Need at least 2 frames
        if len(self.letter_history) < 2:
            return None
        
        # Count occurrences (excluding None)
        letter_counts = {}
        for l in self.letter_history:
            if l is not None:
                letter_counts[l] = letter_counts.get(l, 0) + 1
        
        if not letter_counts:
            return None
        
        # Find most common letter
        best_letter = max(letter_counts, key=letter_counts.get)
        best_count = letter_counts[best_letter]
        
        # Require minimum confidence
        if best_count < self.min_confidence:
            return None
        
        # ============================================
        # DEBOUNCING
        # ============================================
        
        if best_letter == self.last_letter:
            # Same letter - output first time, then every 30 frames
            if not self.letter_output_once:
                self.letter_output_once = True
                self.frames_since_change = 0
                detected = best_letter
            else:
                self.frames_since_change += 1
                if self.frames_since_change >= 30:
                    self.frames_since_change = 0
                    detected = best_letter
                else:
                    detected = None
        else:
            # Different letter
            if self.last_letter is None:
                # First ever letter - output immediately
                self.last_letter = best_letter
                self.frames_since_change = 0
                self.letter_output_once = True
                detected = best_letter
            elif self.frames_since_change >= self.min_frames_between_changes:
                # Enough time passed - allow change
                self.last_letter = best_letter
                self.frames_since_change = 0
                self.letter_output_once = False
                detected = None  # Will output on next frame
            else:
                # Too soon - wait
                self.frames_since_change += 1
                detected = None
        
        # DEBUG output
        if self.debug_mode and detected:
            self._debug_print_raw(cv_positions, curl_angles, states, detected)
        
        return detected
    
    # ===========================================
    # LETTER DETECTION RULES
    # ===========================================
    
    def letter_from_states_enhanced(self, states, cv_positions, curl_angles):
        """
        6 basic letters using RAW pixel distances + IMU angles.
        """
        # Helper: Calculate raw pixel distance
        def px_dist(f1, f2):
            if f1 not in cv_positions or f2 not in cv_positions:
                return 9999
            return self.dist(cv_positions[f1], cv_positions[f2])
        
        # Helper: IMU curl checks
        def is_extended(finger):
            return curl_angles.get(finger, 180) < 30
        
        def is_curled(finger):
            return curl_angles.get(finger, 0) > 90
        
        def is_semi(finger):
            angle = curl_angles.get(finger, 0)
            return 30 <= angle <= 90
        
        # ===========================================
        # 6 LETTERS
        # ===========================================
        
        # 5: All fingers extended AND spread
        if (is_extended("index") and is_extended("middle") and 
            is_extended("ring") and is_extended("pinky") and
            states["thumb"] == "extended"):
            if px_dist("index", "pinky") > 250:
                return "5"
        
        # W: Three fingers extended, pinky curled
        if (is_extended("index") and is_extended("middle") and 
            is_extended("ring") and is_curled("pinky")):
            if px_dist("index", "ring") > 150:
                return "W"
        
        # V: Index, middle extended APART; others curled
        if (is_extended("index") and is_extended("middle") and 
            is_curled("ring") and is_curled("pinky")):
            if px_dist("index", "middle") > 100:
                return "V"
        
        # R: Index, middle extended CROSSED; others curled
        if (is_extended("index") and is_extended("middle") and 
            is_curled("ring") and is_curled("pinky")):
            if px_dist("index", "middle") < 50:
                return "R"
        
        # U: Index, middle extended TOGETHER; others curled
        if (is_extended("index") and is_extended("middle") and 
            is_curled("ring") and is_curled("pinky")):
            if 50 <= px_dist("index", "middle") <= 100:
                return "U"
        
        # X: Index semi-curled (hooked); others curled
        if (is_semi("index") and 
            is_curled("middle") and is_curled("ring") and is_curled("pinky")):
            return "X"
        
        return None
    
    # ===========================================
    # DEBUG OUTPUT
    # ===========================================
    
    def _debug_print_raw(self, cv_positions, curl_angles, states, detected_letter):
        """Debug with RAW pixel distances."""
        print("\n" + "="*70)
        print("🔍 HAND STATE DEBUG (Raw Pixels)")
        print("="*70)
        
        # IMU angles
        print("\n📐 IMU CURL ANGLES:")
        for finger in ['index', 'middle', 'ring', 'pinky']:
            angle = curl_angles.get(finger, 0)
            state = states.get(finger, "unknown")
            marker = "✓EXTEND" if angle < 30 else "✓CURL" if angle > 90 else "~SEMI"
            print(f"  {finger:8s}: {angle:5.1f}° → {state:8s} [{marker}]")
        
        thumb_state = states.get("thumb", "unknown")
        print(f"  thumb   : {thumb_state:8s} (from CV distance)")
        
        # CV distances
        print("\n📏 KEY PIXEL DISTANCES:")
        pairs = [
            ("index", "middle"), 
            ("index", "pinky"), 
            ("index", "ring"), 
            ("thumb", "index"),
            ("ring", "pinky")
        ]
        
        for f1, f2 in pairs:
            if f1 in cv_positions and f2 in cv_positions:
                d = self.dist(cv_positions[f1], cv_positions[f2])
                print(f"  {f1:6s} ↔ {f2:6s}: {d:6.1f}px")
        
        print(f"\n🎯 DETECTED: {detected_letter}")
        print("="*70 + "\n")
