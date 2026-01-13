import cv2
import numpy as np

class CalibrationProcessor:
    def __init__(self, calib, config):
        self.calib = calib
        self.config = config
        self.desk_plane_color = None

    def process(self, frame):
        """
        Process frame for calibration.
        Returns a dictionary with visualization images.
        """
        if frame.color is None:
            return {"color": np.zeros((540, 960, 3), dtype=np.uint8)}
            
        color_bgr = frame.color
        if color_bgr.shape[2] == 4:
            color_bgr = cv2.cvtColor(color_bgr, cv2.COLOR_BGRA2BGR)
        
        # Working on a copy to avoid modifying original frame in place if needed later
        # But for display we usually draw on copy anyway.
        display_img = color_bgr.copy()
        
        fx, fy, cx, cy = frame.color_intrinsics
        camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32)
        dist_coeffs = np.zeros(5, dtype=np.float32)
        
        # Ensure contiguous layout
        color_bgr_contig = np.ascontiguousarray(color_bgr)
            
        board_cfg = self.config.get("calibration_board", {})
        
        plane_cam, pose, num_markers = self.calib.detect_board(color_bgr_contig, camera_matrix, dist_coeffs, board_config=board_cfg)
        
        status_text = f"Markers: {num_markers}"
        text_color = (0, 0, 255)
        
        if plane_cam is not None:
            rvec, tvec = pose
            self.desk_plane_color = plane_cam
            status_text += " | Detected!"
            text_color = (0, 255, 0)
            
            # Visualization
            cv2.drawFrameAxes(display_img, camera_matrix, dist_coeffs, rvec, tvec, 0.1)
        else:
            status_text += " | Searching..."
            
        # Draw text on full resolution image (or we can let UI scale it, but text size might be issue)
        # Let's draw relative to image size
        h, w = display_img.shape[:2]
        cv2.putText(display_img, "CALIBRATION MODE", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 0), 3)
        cv2.putText(display_img, status_text, (30, 120), cv2.FONT_HERSHEY_SIMPLEX, 1.0, text_color, 2)
        
        # We return the full resolution image, let UI handle resizing/fitting
        return {"color": display_img}
