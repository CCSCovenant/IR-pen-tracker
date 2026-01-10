import time
import numpy as np
import cv2
from typing import Optional
import pyrealsense2 as rs

from ..core.interfaces import ICamera
from ..core.types import Frame

class RealSenseCamera(ICamera):
    def __init__(self, depth_width=640, depth_height=480, color_width=640, color_height=480, fps=30, enable_ir=True, preset="high_accuracy"):
        self._pipe: Optional[rs.pipeline] = None
        self._cfg: Optional[rs.config] = None
        self._frame_id = 0
        self._depth_w = int(depth_width)
        self._depth_h = int(depth_height)
        self._color_w = int(color_width)
        self._color_h = int(color_height)
        self._fps = int(fps)
        self._enable_ir = bool(enable_ir)
        self._preset = str(preset).lower()
        self._intrinsics: Optional[np.ndarray] = None
        self._color_intrinsics: Optional[np.ndarray] = None
        self._color_dist_coeffs: Optional[np.ndarray] = None
        self._extrinsics_c2d: Optional[tuple] = None
        self._depth_scale_m: Optional[float] = None

    def open(self) -> bool:
        self._pipe = rs.pipeline()
        self._cfg = rs.config()
        self._cfg.enable_stream(rs.stream.depth, self._depth_w, self._depth_h, rs.format.z16, self._fps)
        self._cfg.enable_stream(rs.stream.color, self._color_w, self._color_h, rs.format.bgr8, self._fps)
        if self._enable_ir:
            try:
                self._cfg.enable_stream(rs.stream.infrared, 1, self._depth_w, self._depth_h, rs.format.y8, self._fps)
            except Exception:
                self._cfg.enable_stream(rs.stream.infrared, self._depth_w, self._depth_h, rs.format.y8, self._fps)
        profile = self._pipe.start(self._cfg)
        dev = profile.get_device()
        depth_sensor = dev.first_depth_sensor()
        try:
            self._depth_scale_m = float(depth_sensor.get_depth_scale())
        except Exception:
            self._depth_scale_m = 0.001
        try:
            if depth_sensor.supports(rs.option.visual_preset):
                preset_map = {
                    "default": 0.0,
                    "hand": 1.0,
                    "high_accuracy": 3.0,
                    "high_density": 4.0
                }
                val = preset_map.get(self._preset, 3.0)
                depth_sensor.set_option(rs.option.visual_preset, val)
        except Exception:
            pass
        pipe_profile = self._pipe.get_active_profile()
        depth_sp = pipe_profile.get_stream(rs.stream.depth).as_video_stream_profile()
        color_sp = pipe_profile.get_stream(rs.stream.color).as_video_stream_profile()
        d_intr = depth_sp.get_intrinsics()
        c_intr = color_sp.get_intrinsics()
        self._intrinsics = np.array([float(d_intr.fx), float(d_intr.fy), float(d_intr.ppx), float(d_intr.ppy)], dtype=np.float32)
        self._color_intrinsics = np.array([float(c_intr.fx), float(c_intr.fy), float(c_intr.ppx), float(c_intr.ppy)], dtype=np.float32)
        self._color_dist_coeffs = np.array(list(c_intr.coeffs), dtype=np.float32)
        ex = color_sp.get_extrinsics_to(depth_sp)
        R = np.array(ex.rotation, dtype=np.float32).reshape(3, 3)
        t_m = np.array(ex.translation, dtype=np.float32).reshape(3)
        t_mm = t_m * 1000.0
        self._extrinsics_c2d = (R, t_mm)
        self._frame_id = 0
        return True

    def get_calibration_data(self):
        return {
            "depth_intrinsics": self._intrinsics.tolist() if self._intrinsics is not None else None,
            "color_intrinsics": self._color_intrinsics.tolist() if self._color_intrinsics is not None else None,
            "color_dist_coeffs": self._color_dist_coeffs.tolist() if self._color_dist_coeffs is not None else None,
            "extrinsics_color_to_depth": {
                "R": self._extrinsics_c2d[0].tolist(),
                "t": self._extrinsics_c2d[1].tolist()
            } if self._extrinsics_c2d else None
        }

    def read_frame(self) -> Optional[Frame]:
        if self._pipe is None:
            return None
        frames = self._pipe.wait_for_frames()
        ts = time.time()
        depth_frame = frames.get_depth_frame()
        if depth_frame is None:
            raise RuntimeError("failed to capture depth image")
        color_frame = frames.get_color_frame()
        ir_frame = None
        if self._enable_ir:
            try:
                ir_frame = frames.get_infrared_frame(1)
            except Exception:
                ir_frame = frames.get_infrared_frame()
        depth = np.asanyarray(depth_frame.get_data())
        if depth.dtype != np.uint16:
            depth = depth.astype(np.uint16)
        color = None
        if color_frame is not None:
            color = np.asanyarray(color_frame.get_data())
        ir = None
        if ir_frame is not None:
            ir_arr = np.asanyarray(ir_frame.get_data())
            if ir_arr.dtype == np.uint8:
                ir = (ir_arr.astype(np.uint16) * 257)
            elif ir_arr.dtype == np.uint16:
                ir = ir_arr
            else:
                ir = ir_arr.astype(np.uint16)
        if self._intrinsics is None:
            raise RuntimeError("intrinsics not loaded")
        frame = Frame(
            timestamp=ts,
            frame_id=self._frame_id,
            color=color,
            depth=depth,
            ir=ir,
            intrinsics=self._intrinsics,
            color_intrinsics=self._color_intrinsics,
            extrinsics_color_to_depth=self._extrinsics_c2d
        )
        self._frame_id += 1
        return frame

    def close(self):
        if self._pipe:
            try:
                self._pipe.stop()
            except Exception:
                pass
            self._pipe = None
