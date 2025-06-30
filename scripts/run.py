#!/usr/bin/env python3

import cv2
import numpy as np
from marker_detector import MarkerDetector

# -----------------------------
# Initialize detector
# -----------------------------
detector = MarkerDetector(
    aruco_dict_type=cv2.aruco.DICT_4X4_50,
    marker_length=0.10
)

# -----------------------------
# Controller parameters
# -----------------------------
desired_distance = 0.5
k_linear = 0.8
k_angular = 0.5
max_linear_speed = 0.05      # m/s
max_angular_speed = 0.2      # rad/s

# Constant rotation speed if no marker
omega_search = np.radians(10)   # about 10 deg/s

# -----------------------------
# Open video
# -----------------------------
video_path = "/home/kimkt0408/bagfiles/2025-06-30-08-20-43.mp4"

cap = cv2.VideoCapture(video_path)

if not cap.isOpened():
    print("Error opening video file.")
    exit(1)

# Optional: create VideoWriter
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

    frame_height, frame_width = frame.shape[:2]
    image_center_x = frame_width / 2

    annotated, detections = detector.detect(frame)

    found_marker = False

    for det in detections:
        marker_id = det["id"]
        if marker_id != 1:
            continue

        found_marker = True

        tvec = det["tvec"]   # x, y, z in camera frame
        corners = det["corners"]

        # Compute marker center in image
        pts = corners.reshape(-1, 2)
        cX = np.mean(pts[:, 0])

        error_px = cX - image_center_x
        omega = -k_angular * error_px / (frame_width / 2)
        omega = np.clip(omega, -max_angular_speed, max_angular_speed)

        # Compute forward distance
        x = tvec[0]    # forward distance
        longitudinal_error = x - desired_distance
        vx = k_linear * longitudinal_error
        vx = np.clip(vx, -max_linear_speed, max_linear_speed)

        print(f"[ID 1 FOUND] | CenterX={cX:.1f}px | Error={error_px:.1f}px")
        print(f"cmd_vel: linear={vx:.3f} m/s, angular={np.degrees(omega):.2f} deg/s")
        print("----------------------------------------------------------")
        break  # Only process the first matching ID=1 marker

    if not found_marker:
        vx = 0.0
        omega = omega_search
        print("[NO ID 1] Rotating in-place to search...")
        print(f"cmd_vel: linear={vx:.3f} m/s, angular={np.degrees(omega):.2f} deg/s")
        print("----------------------------------------------------------")

    # Show result
    cv2.imshow("Video with detected markers", annotated)

    if save_output:
        out.write(annotated)

    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
if save_output:
    out.release()
cv2.destroyAllWindows()
