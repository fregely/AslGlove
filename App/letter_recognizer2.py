# asl/letter_recognizer2.py

import math

class LetterRecognizer2:
    def __init__(self):
        # Stores raw distance samples during calibration
        self.calibration_samples = []

        # Final calibrated average distances
        self.baseline = {
            "thumb_index": None,
            "thumb_middle": None,
            "thumb_ring": None,
            "thumb_pinky": None
        }

        self.is_calibrating = False

    # -----------------------------
    # CALIBRATION WORKFLOW
    # -----------------------------
    def start_calibration(self):
        """
        Call this once when the user begins holding their hand open.
        """
        self.calibration_samples = []
        self.is_calibrating = True
        print("📏 Calibration started — hold your hand open.")

    def add_calibration_sample(self, finger_positions: dict):
        """
        Call this once per frame while the hand is held open.
        Collects thumb→finger distances.
        """
        if not self.is_calibrating:
            return

        required = {"thumb", "index", "middle", "ring", "pinky"}
        if not required.issubset(finger_positions.keys()):
            return

        thumb = finger_positions["thumb"]

        sample = {
            "thumb_index":  self.dist(thumb, finger_positions["index"]),
            "thumb_middle": self.dist(thumb, finger_positions["middle"]),
            "thumb_ring":   self.dist(thumb, finger_positions["ring"]),
            "thumb_pinky":  self.dist(thumb, finger_positions["pinky"])
        }

        self.calibration_samples.append(sample)

    def end_calibration(self):
        """
        Computes the average distances and stores them as baseline.
        """
        if not self.calibration_samples:
            print("⚠ No calibration samples collected.")
            return

        # Average each distance across all samples
        totals = defaultdict(float)
        n = len(self.calibration_samples)

        for sample in self.calibration_samples:
            for key, value in sample.items():
                totals[key] += value

        for key in self.baseline:
            self.baseline[key] = totals[key] / n

        self.is_calibrating = False
        print("✅ Calibration complete.")
        print("Baseline distances:", self.baseline)

    # -----------------------------
    # CLASSIFICATION LOGIC (later)
    # -----------------------------
    def classify(self, finger_positions):
        """
        Placeholder — after calibration, you’ll compare new distances
        against the baseline to detect a letter.
        """
        if None in self.baseline.values():
            print("❌ ERROR: Cannot classify — run calibration first.")
            return None

        thumb = finger_positions["thumb"]

        # Compute distances for current frame
        distances = {
            "thumb_index":  self.dist(thumb, finger_positions["index"]),
            "thumb_middle": self.dist(thumb, finger_positions["middle"]),
            "thumb_ring":   self.dist(thumb, finger_positions["ring"]),
            "thumb_pinky":  self.dist(thumb, finger_positions["pinky"])
        }

        calc = 3 #calc is where we will insert the calculated value for each sign
        offset = 1

        # Letter "A": all fingers curled, thumb on side
        if (abs(distances["thumb_index"] - calc) < offset):
            if (abs(distances["thumb_middle"] - calc) < offset):
                if (abs(distances["thumb_ring"] - calc) < offset):
                    if (abs(distances["thumb_pinky"] - calc) < offset):
                        return "A"

            
        # Letter "B": all fingers extended upward, thumb across palm
        if (abs(distances["thumb_index"] - calc) < offset):
            if (abs(distances["thumb_middle"] - calc) < offset):
                if (abs(distances["thumb_ring"] - calc) < offset):
                    if (abs(distances["thumb_pinky"] - calc) < offset):
                        return "B"

        # Letter "C": curved spatial arc between thumb and pinky
        if (abs(distances["thumb_index"] - calc) < offset):
            if (abs(distances["thumb_middle"] - calc) < offset):
                if (abs(distances["thumb_ring"] - calc) < offset):
                    if (abs(distances["thumb_pinky"] - calc) < offset):
                        return "C"
        
        # Letter "D": pointer extended, others making o shape with thumb
        if (abs(distances["thumb_index"] - calc) < offset):
            if (abs(distances["thumb_middle"] - calc) < offset):
                if (abs(distances["thumb_ring"] - calc) < offset):
                    if (abs(distances["thumb_pinky"] - calc) < offset):
                        return "D"
            
        # Letter "E": all fingers bent, varies based on person
        if (abs(distances["thumb_index"] - calc) < offset):
            if (abs(distances["thumb_middle"] - calc) < offset):
                if (abs(distances["thumb_ring"] - calc) < offset):
                    if (abs(distances["thumb_pinky"] - calc) < offset):
                        return "E"

        # Letter "F": middle, ring, and pinky extended, pointer and thumb curled together
        if (abs(distances["thumb_index"] - calc) < offset):
            if (abs(distances["thumb_middle"] - calc) < offset):
                if (abs(distances["thumb_ring"] - calc) < offset):
                    if (abs(distances["thumb_pinky"] - calc) < offset):
                        return "F"

        # Letter "G": pointer finger extended to the side
            #need to check if it's to the side versus original orientation of hand?
            #middle, ring, pinky curled into hand
            if (abs(distances["thumb_index"] - calc) < offset):
                if (abs(distances["thumb_middle"] - calc) < offset):
                    if (abs(distances["thumb_ring"] - calc) < offset):
                        if (abs(distances["thumb_pinky"] - calc) < offset):
                            return "G"

        # Letter "H": pointer and middle extended to the side
            #ring and pinky curled into hand
            if (abs(distances["thumb_index"] - calc) < offset):
                if (abs(distances["thumb_middle"] - calc) < offset):
                    if (abs(distances["thumb_ring"] - calc) < offset):
                        if (abs(distances["thumb_pinky"] - calc) < offset):
                            return "H"

        # Letter "I" and "J": only pinky extended, pointer, middle, ring curled into hand
            #no idea how to do J, need to watch the SEQUENTIAL datas if pinky swings down
            if (abs(distances["thumb_index"] - calc) < offset):
                if (abs(distances["thumb_middle"] - calc) < offset):
                    if (abs(distances["thumb_ring"] - calc) < offset):
                        if (abs(distances["thumb_pinky"] - calc) < offset):
                            return "I"

        # Letter "K": pointer and middle extended and far apart
            #the difference between this and V is that the thumb is also extended 
            # (in between pointer and middle)
            if (abs(distances["thumb_index"] - calc) < offset):
                if (abs(distances["thumb_middle"] - calc) < offset):
                    if (abs(distances["thumb_ring"] - calc) < offset):
                        if (abs(distances["thumb_pinky"] - calc) < offset):
                            return "K"

        # Letter "L": thumb and index extended, others bent
        if (abs(distances["thumb_index"] - calc) < offset):
            if (abs(distances["thumb_middle"] - calc) < offset):
                if (abs(distances["thumb_ring"] - calc) < offset):
                    if (abs(distances["thumb_pinky"] - calc) < offset):
                        return "L"
            
        # Letter "M": 
        if (abs(distances["thumb_index"] - calc) < offset):
            if (abs(distances["thumb_middle"] - calc) < offset):
                if (abs(distances["thumb_ring"] - calc) < offset):
                    if (abs(distances["thumb_pinky"] - calc) < offset):
                        return "M"

        # Letter "N":
        if (abs(distances["thumb_index"] - calc) < offset):
            if (abs(distances["thumb_middle"] - calc) < offset):
                if (abs(distances["thumb_ring"] - calc) < offset):
                    if (abs(distances["thumb_pinky"] - calc) < offset):
                        return "N"

        # Letter "O": all fingers curled, close to thumb
        if (abs(distances["thumb_index"] - calc) < offset):
            if (abs(distances["thumb_middle"] - calc) < offset):
                if (abs(distances["thumb_ring"] - calc) < offset):
                    if (abs(distances["thumb_pinky"] - calc) < offset):
                        return "O"

        # Letter "P":
        if (abs(distances["thumb_index"] - calc) < offset):
            if (abs(distances["thumb_middle"] - calc) < offset):
                if (abs(distances["thumb_ring"] - calc) < offset):
                    if (abs(distances["thumb_pinky"] - calc) < offset):
                        return "P"

        # Letter "Q":
        if (abs(distances["thumb_index"] - calc) < offset):
            if (abs(distances["thumb_middle"] - calc) < offset):
                if (abs(distances["thumb_ring"] - calc) < offset):
                    if (abs(distances["thumb_pinky"] - calc) < offset):
                        return "Q"

        # Letter "R": pointer and middle extended up together, curled around each other
            #other fingers bent
            #NEED TO DECIPER FROM U
            if (abs(distances["thumb_index"] - calc) < offset):
                if (abs(distances["thumb_middle"] - calc) < offset):
                    if (abs(distances["thumb_ring"] - calc) < offset):
                        if (abs(distances["thumb_pinky"] - calc) < offset):
                            return "R"

        # Letter "S":
        if (abs(distances["thumb_index"] - calc) < offset):
            if (abs(distances["thumb_middle"] - calc) < offset):
                if (abs(distances["thumb_ring"] - calc) < offset):
                    if (abs(distances["thumb_pinky"] - calc) < offset):
                        return "S"

        # Letter "T":
        if (abs(distances["thumb_index"] - calc) < offset):
            if (abs(distances["thumb_middle"] - calc) < offset):
                if (abs(distances["thumb_ring"] - calc) < offset):
                    if (abs(distances["thumb_pinky"] - calc) < offset):
                        return "T"

        # Letter "U": literally the same as R but pointer and middle not curled around each other
        if (abs(distances["thumb_index"] - calc) < offset):
            if (abs(distances["thumb_middle"] - calc) < offset):
                if (abs(distances["thumb_ring"] - calc) < offset):
                    if (abs(distances["thumb_pinky"] - calc) < offset):
                        return "U"

        # Letter "V": ring and pinky curled, pointer and middle extended but far apart
        if (abs(distances["thumb_index"] - calc) < offset):
            if (abs(distances["thumb_middle"] - calc) < offset):
                if (abs(distances["thumb_ring"] - calc) < offset):
                    if (abs(distances["thumb_pinky"] - calc) < offset):
                        return "V"

        # Letter "W": pinky curled, pointer, middle, and ring extended
        if (abs(distances["thumb_index"] - calc) < offset):
            if (abs(distances["thumb_middle"] - calc) < offset):
                if (abs(distances["thumb_ring"] - calc) < offset):
                    if (abs(distances["thumb_pinky"] - calc) < offset):
                        return "W"

        # Letter "X": pointer up but bent (like a hook), all others curled
            #need to deciper from D and X
            #maybe have the initial stretched out hand position, that one is "D", and if
            #pointer finger is lower than that but still extended it's "X"
            if (abs(distances["thumb_index"] - calc) < offset):
                if (abs(distances["thumb_middle"] - calc) < offset):
                    if (abs(distances["thumb_ring"] - calc) < offset):
                        if (abs(distances["thumb_pinky"] - calc) < offset):
                            return "X"

        # Letter "Y": pointer, middle, ring curled, thumb and pinky extended
        if (abs(distances["thumb_index"] - calc) < offset):
            if (abs(distances["thumb_middle"] - calc) < offset):
                if (abs(distances["thumb_ring"] - calc) < offset):
                    if (abs(distances["thumb_pinky"] - calc) < offset):
                        return "Y"

        # Letter "Z": check with D
        if (abs(distances["thumb_index"] - calc) < offset):
            if (abs(distances["thumb_middle"] - calc) < offset):
                if (abs(distances["thumb_ring"] - calc) < offset):
                    if (abs(distances["thumb_pinky"] - calc) < offset):
                        return "Z"


        return None

    # -----------------------------
    # UTILITIES
    # -----------------------------
    def dist(self, p1, p2):
        return math.dist(p1, p2)
