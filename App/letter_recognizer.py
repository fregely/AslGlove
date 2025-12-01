# asl/letter_recognizer.py
# pylint: disable=E1101
# mypy: ignore-errors

import math

class LetterRecognizer:
    """
    Receives:
        - finger_positions: dict(finger -> (x,y) in meters or pixels)
        - imu_orientations: optional dict(finger -> (roll,pitch,yaw))

    Returns:
        letter: str or None
    """

    def classify(self, finger_positions: dict) -> str | None:
        """ Classify letter based on finger positions.
            Very rough heuristic rules for demo purposes."""
        if len(finger_positions) < 5:
            return None  # incomplete vision

        thumb = finger_positions['thumb']
        index = finger_positions['index']
        middle = finger_positions['middle']
        ring = finger_positions['ring']
        pinky = finger_positions['pinky']

        # Compute distances between thumb and others
        d_thumb_index  = self.dist(thumb, index)
        d_thumb_middle = self.dist(thumb, middle)

        # Average finger height (x or y depends on your camera axis)
        # Let's use y: lower y means finger is higher on screen
        y_vals = [p[1] for p in finger_positions.values()]
        avg_y = sum(y_vals)/len(y_vals)

        # ——————————————————————
        # SAMPLE RULES (very rough!)
        # you will refine them with calibration data
        # ——————————————————————

        # Letter "A": all fingers curled, thumb on side
        if (self.is_near_thumb(thumb, index) and
            self.is_near_thumb(thumb, middle)):
            if self.fingers_curled(y_vals):
                return "A"

        # Letter "L": thumb and index extended, others bent
        if self.is_far(thumb, index) and self.is_bent(middle):
            if self.is_bent(ring) and self.is_bent(pinky):
                return "L"

        # Letter "B": all fingers extended upward, thumb across palm
        if self.all_extended(y_vals) and self.is_near_thumb(thumb, index):
            return "B"

        # Letter "C": curved spatial arc between thumb and pinky
        if self.c_curve_shape(thumb, index, middle, ring, pinky):
            return "C"

        return None


    # -----------------------------
    # Utility helpers
    # -----------------------------
    def dist(self, p1, p2) -> float:
        return math.dist(p1, p2)

    def is_near_thumb(self, thumb, other, thresh=0.025):
        return self.dist(thumb, other) < thresh

    def is_far(self, p1, p2, thresh=0.05):
        return self.dist(p1, p2) > thresh

    def fingers_curled(self, y_vals):
        """
        heuristic: curled fingers cluster at similar y
        (their tips are lower than fully extended)
        """
        var = max(y_vals)-min(y_vals)
        return var < 0.02

    def is_bent(self, finger_pos, threshold=0.015):
        """
        finger considered bent if it's 'lower' relative to others
        """
        return finger_pos[1] > threshold

    def all_extended(self, y_vals, threshold=0.01):
        return all(y < threshold for y in y_vals)

    def c_curve_shape(self, thumb, idx, mid, ring, pinky):
        return (
            self.dist(thumb, pinky) > 0.04 and
            abs(self.dist(idx, mid) - self.dist(ring, pinky)) < 0.02
        )