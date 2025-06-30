import cv2
import numpy as np

class MarkerDetector:
    def __init__(self, 
                 aruco_dict_type=cv2.aruco.DICT_4X4_50,
                 marker_length=0.1,
                 camera_matrix=None,
                 dist_coeffs=None):
        
        self.image_width = 640
        self.image_height = 480
        self.marker_length = marker_length
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
        self.detector_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.detector_params)
        
        # TODO: Modify the camera matrix and distortion coefficients
        if camera_matrix is None:
            camera_matrix = np.array([[600, 0, 320],
                                      [0, 600, 240],
                                      [0, 0, 1]], dtype=np.float32)
        if dist_coeffs is None:
            dist_coeffs = np.zeros((5, 1))
        
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs

    def detect(self, image):
        """
        Detect markers and estimate poses in an image.
        
        Returns:
          - annotated_image: The image with boxes drawn
          - detections: list of dicts with fields:
              - id
              - tvec
              - rvec
              - corners
        """
        annotated = image.copy()
        corners, ids, _ = self.detector.detectMarkers(image)
        
        detections = []
        
        if ids is not None:
            for i in range(len(ids)):
                marker_corners = corners[i]
                marker_id = int(ids[i][0])
                
                # Draw polyline
                pts = marker_corners.reshape(-1, 2).astype(int)
                cv2.polylines(annotated, [pts], isClosed=True, color=(0, 255, 0), thickness=2)
                
                cX = int(np.mean(pts[:, 0]))
                cY = int(np.mean(pts[:, 1]))
                cv2.putText(
                    annotated,
                    f"ID:{marker_id}",
                    (cX, cY - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 0, 255),
                    2
                )
                
                # Estimate pose
                half_size = self.marker_length / 2
                obj_points = np.array([
                    [-half_size,  half_size, 0],
                    [ half_size,  half_size, 0],
                    [ half_size, -half_size, 0],
                    [-half_size, -half_size, 0]
                ], dtype=np.float32)
                
                img_points = marker_corners.reshape(-1, 2)
                
                retval, rvec, tvec = cv2.solvePnP(
                    obj_points,
                    img_points,
                    self.camera_matrix,
                    self.dist_coeffs
                )
                tvec = tvec.flatten()
                rvec = rvec.flatten()
                
                detections.append({
                    "id": marker_id,
                    "tvec": tvec,
                    "rvec": rvec,
                    "corners": marker_corners
                })
        
        return annotated, detections

    def compute_velocity(self, det, 
                        desired_distance,
                        k_linear,
                        k_angular,
                        max_linear_speed,
                        max_angular_speed):
        """
        Compute linear and angular velocities from a marker detection.
        
        Args:
            det: detection dict (contains "tvec" and "corners")
            desired_distance: target distance to the marker
            k_linear: linear control gain
            k_angular: angular control gain
            max_linear_speed: max linear velocity (m/s)
            max_angular_speed: max angular velocity (rad/s)
        
        Returns:
            vx: linear velocity (m/s)
            omega: angular velocity (rad/s)
            extra_info: dict with debug info
        """
        tvec = det["tvec"]
        corners = det["corners"]

        # Compute marker center in image
        pts = corners.reshape(-1, 2)
        cX = np.mean(pts[:, 0])

        image_center_x = self.image_width / 2
        error_px = cX - image_center_x

        # Angular velocity from pixel error
        omega = -k_angular * error_px / (self.image_width / 2)
        omega = np.clip(omega, -max_angular_speed, max_angular_speed)

        # Linear velocity from distance along Z axis
        z = tvec[2]
        longitudinal_error = z - desired_distance
        vx = k_linear * longitudinal_error
        vx = np.clip(vx, -max_linear_speed, max_linear_speed)

        extra_info = {
            "cX": cX,
            "error_px": error_px,
            "z": z,
            "longitudinal_error": longitudinal_error
        }

        return vx, omega, extra_info
