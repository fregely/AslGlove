# asl/letter_recognizer.py

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
            
        # Letter "B": all fingers extended upward, thumb across palm
        if self.all_extended(y_vals) and self.is_near_thumb(thumb, index):
            return "B"

        # Letter "C": curved spatial arc between thumb and pinky
        if self.c_curve_shape(thumb, index, middle, ring, pinky):
            return "C"
        
        # Letter "D": pointer extended, others making o shape with thumb
        if (self.finger_extended(index, y_vals) and 
            self.is_near_thumb(thumb, middle) and self.is_near_thumb(thumb, ring) 
            and self.is_near_thumb(thumb, pinky)):
            if self.fingers_curled(y_vals):
                return "D"
            
        # Letter "E": all fingers bent, varies based on person

        # Letter "F": middle, ring, and pinky extended, pointer and thumb curled together

        # Letter "G": pointer finger extended to the side
            #need to check if it's to the side versus original orientation of hand?
            #middle, ring, pinky curled into hand

        # Letter "H": pointer and middle extended to the side
            #ring and pinky curled into hand

        # Letter "I" and "J": only pinky extended, pointer, middle, ring curled into hand
            #no idea how to do J, need to watch the SEQUENTIAL datas if pinky swings down

        # Letter "K": pointer and middle extended and far apart
            #the difference between this and V is that the thumb is also extended 
            # (in between pointer and middle)

        # Letter "L": thumb and index extended, others bent
        if self.is_far(thumb, index) and self.is_bent(middle):
            if self.is_bent(ring) and self.is_bent(pinky):
                return "L"
            
        # Letter "M": 

        # Letter "N":

        # Letter "O": all fingers curled, close to thumb
        if (self.fingers_curled(y_vals) and self.is_near_thumb(thumb, index) and 
            self.is_near_thumb(thumb, middle) and self.is_near_thumb(thumb, ring)
            and self.is_near_thumb(thumb, pinky)):
            return "O"

        # Letter "P":

        # Letter "Q":

        # Letter "R": pointer and middle extended up together, curled around each other
            #other fingers bent
            #NEED TO DECIPER FROM U
        if self.is_bent(ring) and self.is_bent(pinky) and self.is_close(index, middle):
            return "R"

        # Letter "S":

        # Letter "T":

        # Letter "U": literally the same as R but pointer and middle not curled around each other
        if self.is_bent(ring) and self.is_bent(pinky) and self.is_close(index, middle):
            return "U"

        # Letter "V": ring and pinky curled, pointer and middle extended but far apart

        # Letter "W": pinky curled, pointer, middle, and ring extended

        # Letter "X": pointer up but bent (like a hook), all others curled
            #need to deciper from D and X
            #maybe have the initial stretched out hand position, that one is "D", and if
            #pointer finger is lower than that but still extended it's "X"

        # Letter "Y": pointer, middle, ring curled, thumb and pinky extended
        if self.finger_extended(thumb, y_vals) and self.finger_extended(pinky, y_vals):
            if self.is_bent(index) and self.is_bent(middle) and self.is_bent(ring):
                return "Y"

        # Letter "Z": check with D


        return None


    # -----------------------------
    # Utility helpers
    # -----------------------------
    def dist(self, p1, p2):
        return math.dist(p1, p2)

    def is_near_thumb(self, thumb, other, thresh=0.025):
        return self.dist(thumb, other) < thresh

    def is_far(self, p1, p2, thresh=0.05):
        return self.dist(p1, p2) > thresh
    
    def is_close(self, p1, p2, thresh=0.025):
        return self.dist(p1,p2) < thresh

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
    
    def finger_extended(self, finger, all_fingers, diff=0.015):
        """
        finger : (x,y,z) tuple for the finger you're checking
        all_fingers : list of (x,y,z) for index, middle, ring, pinky
        diff : minimum y separation to count as extended
        """
        finger_y = finger[1]

        ys = [f[1] for f in all_fingers]
        avg_other = sum(ys) / len(ys)

        return (avg_other - finger_y) > diff
