"""
Simple OpenCV test script for ASL Glove CV work
Captures frames, converts to grayscale, and shows them live
"""

# Suppress linter false positives about cv2 dynamic attributes
# pylint: disable=E1101

import cv2

def main() -> None:
    """Main loop to capture and display frames from webcam."""
    
    # Print version + check required members exist
    print("OpenCV version:", cv2.__version__)
    print("Has VideoCapture:", hasattr(cv2, "VideoCapture"))
    print("Has SimpleBlobDetector:", hasattr(cv2, "SimpleBlobDetector_create"))

    # Open default camera (index 0)
    cap = cv2.VideoCapture(0)

    # Optional: set resolution & fps
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_FPS, 60)

    if not cap.isOpened():
        print("❌ Error: Could not open camera.")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            print("❌ Error: Failed to grab frame.")
            break

        # Convert to grayscale + blur
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Show both raw and processed views
        cv2.imshow("Raw Frame", frame)
        cv2.imshow("Gray + Blur", blurred)

        # Press 'q' to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
    