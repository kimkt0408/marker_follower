#!/usr/bin/env python3

import cv2
from marker_detector import MarkerDetector

# -----------------------------
# Initialize detector
# -----------------------------
detector = MarkerDetector(
    aruco_dict_type=cv2.aruco.DICT_4X4_50,
    marker_length=0.10
)

# -----------------------------
# Open video
# -----------------------------
video_path = "/home/kimkt0408/bagfiles/2025-06-30-08-20-43.mp4"

cap = cv2.VideoCapture(video_path)

# Check if opened successfully
if not cap.isOpened():
    print("Error opening video file.")
    exit(1)

# Optional: create VideoWriter to save output video
save_output = False
if save_output:
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(
        "annotated_output.mp4",
        fourcc,
        cap.get(cv2.CAP_PROP_FPS),
        (
            int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
            int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        )
    )

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    annotated, detections = detector.detect(frame)
    
    # Print detection info
    for det in detections:
        print(f"ID={det['id']}, tvec={det['tvec']}")
    
    # Show result
    cv2.imshow("Video with detected markers", annotated)
    
    if save_output:
        out.write(annotated)
    
    # Press ESC to quit
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
if save_output:
    out.release()
cv2.destroyAllWindows()
