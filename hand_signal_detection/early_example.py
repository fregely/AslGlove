import numpy as np

# Example IMU packet format (after parsing)
imu_data = {
    "thumb":  (10, -20, 5),   # pitch, roll, yaw
    "index":  (45, -10, 0),
    "middle": (60, -5, 2),
    "ring":   (55, -8, 1),
    "pinky":  (40, -12, 3),
}

# ASL letter templates (example threshold-based dictionary)
ASL_DICT = {
    "A": {"thumb": (0, 0, 0), "index": (90, 0, 0), "middle": (90, 0, 0), "ring": (90, 0, 0), "pinky": (90, 0, 0)},
    "B": {"thumb": (0, 0, 0), "index": (0, 0, 0), "middle": (0, 0, 0), "ring": (0, 0, 0), "pinky": (0, 0, 0)},
    "C": {"thumb": (45, 0, 0), "index": (45, 0, 0), "middle": (45, 0, 0), "ring": (45, 0, 0), "pinky": (45, 0, 0)},
}

def match_letter(data: dict[str, tuple[float, float, float]]) -> tuple[str, float]:
    best_letter: str = ""
    best_score: float = float("inf")

    for letter, template in ASL_DICT.items():
        letter_score: float = 0.0
        for finger in data.keys():
            diff: float = float(np.linalg.norm(np.array(data[finger]) - np.array(template[finger])))
            letter_score += diff
        if letter_score < best_score:
            best_score = letter_score
            best_letter = letter

    return best_letter, best_score

detected_letter, score = match_letter(imu_data)
print(f"Detected letter: {detected_letter} (score={score:.2f})")