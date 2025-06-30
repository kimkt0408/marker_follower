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
max_linear_speed = 0.05                 # m/s
max_angular_speed = np.radians(10)      # rad/s

# Constant rotation speed if no marker
omega_search = np.radians(1)   # about 1 deg/s

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

        vx, omega, info = detector.compute_velocity(
            det,
            desired_distance,
            k_linear,
            k_angular,
            max_linear_speed,
            max_angular_speed
        )
        
        # Optional debug
        status_text = f"[ID 1 FOUND] CenterX={info['cX']:.1f}px | Error={info['error_px']:.1f}px"
        vel_text = f"cmd_vel: linear={vx:.3f} m/s, angular={np.degrees(omega):.2f} deg/s"

        cv2.putText(
            annotated, status_text, (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2
        )
        cv2.putText(
            annotated, vel_text, (10, 60),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2
        )
        break

    if not found_marker:
        vx = 0.0
        omega = omega_search

        status_text = "[NO ID 1] Rotating in-place to search..."
        vel_text = f"cmd_vel: linear={vx:.3f} m/s, angular={np.degrees(omega):.2f} deg/s"

        cv2.putText(
            annotated, status_text, (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2
        )
        cv2.putText(
            annotated, vel_text, (10, 60),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2
        )

    # Draw integrated velocity arrow
    center_pt = (int(image_center_x), int(frame_height / 2))

    if abs(vx) < 1e-5:
        # Draw small dot if stationary
        cv2.circle(
            annotated,
            center_pt,
            radius=5,
            color=(0, 0, 255),
            thickness=-1
        )
    else:
        if abs(omega) < 1e-5:
            # Draw straight arrow
            length_px = int((vx / max_linear_speed) * 200)
            end_pt = (center_pt[0], center_pt[1] - length_px)
            cv2.arrowedLine(
                annotated, center_pt, end_pt,
                color=(0, 0, 255), thickness=3, tipLength=0.3
            )
        else:
            R = vx / omega if omega != 0 else 1e6
            R_px = np.clip(abs(R) * 400, 120, 1200)  # radius in pixels

            curve_sign = -np.sign(omega)

            # Compute arc angle proportional to |omega|
            scale_factor = np.radians(45) / max_angular_speed
            arc_angle = np.clip(scale_factor * abs(omega), np.radians(5), np.radians(90))

            num_points = 30
            angles = np.linspace(0, arc_angle, num_points)

            points = []
            for theta in angles:
                x = curve_sign * R_px * (1 - np.cos(theta))
                y = -R_px * np.sin(theta)
                point = np.array(center_pt) + np.array([x, y])
                points.append(point.astype(int))

            points = np.array(points, dtype=int).reshape(-1, 1, 2)

            # Draw curve
            cv2.polylines(
                annotated, [points],
                isClosed=False,
                color=(0, 0, 255),
                thickness=3
            )

            # Draw arrowhead
            arrow_start = tuple(points[-2, 0])
            arrow_end = tuple(points[-1, 0])
            cv2.arrowedLine(
                annotated,
                arrow_start,
                arrow_end,
                color=(0, 0, 255),
                thickness=3,
                tipLength=0.3
            )

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
