# asl/letter_recognizer2.py

import math
import statistics

class LetterRecognizer2:

    def __init__(self):
        # Stores normalized distance baselines for open hand
        self.open_calibration = {}   # {("thumb","index"): avg_distance, ...}
        self.finger_names = ["thumb", "index", "middle", "ring", "pinky"]


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
    # 1. Calibration: open hand
    # -------------------------------
    async def calibrate_open_hand(self, client, samples=50):
        """
        Collects fingertip positions while the user holds an open hand
        and computes average distances between all fingertip pairs.
        """

        print("\n--- OPEN HAND CALIBRATION ---")
        print("Please hold your hand completely open and still.")
        print(f"Collecting {samples} samples...")
        
        # Temporary storage for distances
        collected = {pair: [] for pair in self._finger_pairs()}

        count = 0
        while count < samples:
            packet = await self._get_finger_positions(client)
            if packet is None:
                continue

            # Compute distances for this sample
            for (f1, f2) in self._finger_pairs():
                d = self.dist(packet[f1], packet[f2])
                collected[(f1, f2)].append(d)

            count += 1

        # Average distances
        for pair in collected:
            self.open_calibration[pair] = statistics.mean(collected[pair])

        print("✔ Open-hand calibration complete!\n")


    # -------------------------------
    # Helper: list of all pair combinations
    # -------------------------------
    def _finger_pairs(self):
        pairs = []
        for i in range(len(self.finger_names)):
            for j in range(i + 1, len(self.finger_names)):
                pairs.append((self.finger_names[i], self.finger_names[j]))
        return pairs


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
        normalized = {}

        for (f1, f2) in self._finger_pairs():
            cur_d = self.dist(packet[f1], packet[f2])
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
        """

        normalized = self.compute_normalized_distances(packet)
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

    def assign_letters(self, states):

        # Letter "A": all fingers curled, thumb on side
        if states["index"] == "curled":
            if states["middle"] == "curled":
                if states["ring"] == "curled":
                    if states["pinky"] == "curled":
                        return "A"
                
        # Letter "B": all fingers extended upward, thumb across palm
            

        # Letter "C": curved spatial arc between thumb and pinky
            
            
        # Letter "D": pointer extended, others making o shape with thumb
        
            
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
        
            
        # Letter "M": 

        # Letter "N":

        # Letter "O": all fingers curled, close to thumb
        

        # Letter "P":

        # Letter "Q":

        # Letter "R": pointer and middle extended up together, curled around each other
            #other fingers bent
            #NEED TO DECIPER FROM U
        

        # Letter "S":

        # Letter "T":

        # Letter "U": literally the same as R but pointer and middle not curled around each other
        

        # Letter "V": ring and pinky curled, pointer and middle extended but far apart

        # Letter "W": pinky curled, pointer, middle, and ring extended

        # Letter "X": pointer up but bent (like a hook), all others curled
            #need to deciper from D and X
            #maybe have the initial stretched out hand position, that one is "D", and if
            #pointer finger is lower than that but still extended it's "X"

        # Letter "Y": pointer, middle, ring curled, thumb and pinky extended
        

        # Letter "Z": check with D

        return None