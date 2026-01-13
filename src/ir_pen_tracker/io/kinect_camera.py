import time
import numpy as np
from typing import Optional
import pyk4a
from pyk4a import PyK4A, Config, ColorResolution, ImageFormat, FPS, DepthMode

from ..core.interfaces import ICamera
from ..core.types import Frame

class KinectCamera(ICamera):
    def __init__(self, use_raw_color=False, 
                 color_resolution=ColorResolution.RES_720P,
                 camera_fps=FPS.FPS_30,
                 depth_mode=DepthMode.NFOV_UNBINNED):
        self._cam: Optional[PyK4A] = None
        self._intrinsics: Optional[np.ndarray] = None
        self._frame_id = 0
        self._use_raw_color = use_raw_color
        
        self._color_resolution = color_resolution
        self._camera_fps = camera_fps
        self._depth_mode = depth_mode

        self._color_intrinsics: Optional[np.ndarray] = None
        self._color_dist_coeffs: Optional[np.ndarray] = None
        self._extrinsics_c2d: Optional[tuple] = None # (R, t) Color -> Depth
        self._extrinsics_d2c: Optional[tuple] = None # (R, t) Depth -> Color

    def open(self) -> bool:
        cfg = Config(
            color_resolution=self._color_resolution,
            # transformed_color requires BGRA32 input
            color_format=ImageFormat.COLOR_BGRA32,
            camera_fps=self._camera_fps,
            depth_mode=self._depth_mode,
            synchronized_images_only=True,
        )
        self._cam = PyK4A(cfg)
        self._cam.start()
        
        calib = self._cam.calibration
        # Use DEPTH camera intrinsics because we are working in Depth Camera Viewpoint
        mat = calib.get_camera_matrix(pyk4a.CalibrationType.DEPTH)
        # mat is 3x3 [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]
        self._intrinsics = np.array([mat[0, 0], mat[1, 1], mat[0, 2], mat[1, 2]], dtype=np.float32)
        
        # Always load Color Intrinsics and Extrinsics if available
        mat_c = calib.get_camera_matrix(pyk4a.CalibrationType.COLOR)
        self._color_intrinsics = np.array([mat_c[0, 0], mat_c[1, 1], mat_c[0, 2], mat_c[1, 2]], dtype=np.float32)
        self._color_dist_coeffs = calib.get_distortion_coefficients(pyk4a.CalibrationType.COLOR)
        self._extrinsics_c2d = calib.get_extrinsic_parameters(
            pyk4a.CalibrationType.COLOR, 
            pyk4a.CalibrationType.DEPTH
        )
        self._extrinsics_d2c = calib.get_extrinsic_parameters(
            pyk4a.CalibrationType.DEPTH, 
            pyk4a.CalibrationType.COLOR
        )
            
        self._frame_id = 0
        return True

    def get_calibration_data(self):
        """Returns dictionary with calibration data."""
        return {
            "depth_intrinsics": self._intrinsics.tolist() if self._intrinsics is not None else None,
            "color_intrinsics": self._color_intrinsics.tolist() if self._color_intrinsics is not None else None,
            "color_dist_coeffs": self._color_dist_coeffs.tolist() if self._color_dist_coeffs is not None else None,
            "extrinsics_color_to_depth": {
                "R": self._extrinsics_c2d[0].tolist(),
                "t": self._extrinsics_c2d[1].tolist()
            } if self._extrinsics_c2d else None,
            "extrinsics_depth_to_color": {
                "R": self._extrinsics_d2c[0].tolist(),
                "t": self._extrinsics_d2c[1].tolist()
            } if self._extrinsics_d2c else None
        }

    def read_frame(self) -> Optional[Frame]:
        if self._cam is None:
            return None
        cap = self._cam.get_capture()
        ts = time.time()
        
        # 1. Get Raw Depth and IR (Aligned by nature)
        depth = cap.depth
        if depth is None:
            raise RuntimeError("failed to capture depth image")
            
        ir = cap.ir
        if ir is not None:
            # Keep raw uint16 values (>32767 is valid)
            ir = ir.astype(np.uint16)
            
        # 2. Get Color (ALWAYS RAW)
        # Use RAW Color Image
        raw_color_bgra = cap.color
        if raw_color_bgra is None:
            # If color is missing for some reason
            bgr = None 
        else:
            bgr = raw_color_bgra[:, :, :3]

        if self._intrinsics is None:
            raise RuntimeError("intrinsics not loaded")
            
        frame = Frame(
            timestamp=ts,
            frame_id=self._frame_id,
            color=bgr,
            depth=depth,
            ir=ir,
            intrinsics=self._intrinsics,
            color_intrinsics=self._color_intrinsics,
            extrinsics_color_to_depth=self._extrinsics_c2d
        )
        self._frame_id += 1
        return frame

    def close(self):
        if self._cam:
            self._cam.stop()
            self._cam = None
