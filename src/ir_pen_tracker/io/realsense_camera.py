import time
import numpy as np
from typing import Any, Optional, cast
import pyrealsense2 as rs

from ..core.interfaces import ICamera
from ..core.types import Frame

rs = cast(Any, rs)

class RealSenseCamera(ICamera):
    def __init__(self, depth_width=640, depth_height=480, color_width=640, color_height=480, fps=30, enable_ir=True, preset="high_accuracy", laser_power: Optional[float] = None, exposure: Optional[float] = None):
        self._pipe: Optional[rs.pipeline] = None
        self._depth_sensor: Optional[rs.depth_sensor] = None
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
        self._rs_depth_intrinsics: Optional[rs.intrinsics] = None
        self._rs_color_intrinsics: Optional[rs.intrinsics] = None
        self._rs_ir_left_intrinsics: Optional[rs.intrinsics] = None
        self._rs_ir_right_intrinsics: Optional[rs.intrinsics] = None
        self._color_dist_coeffs: Optional[np.ndarray] = None
        self._extrinsics_c2d: Optional[tuple] = None
        self._extrinsics_d2c: Optional[tuple] = None
        self._depth_scale_m: Optional[float] = None
        self._ir_left_intrinsics: Optional[np.ndarray] = None
        self._ir_right_intrinsics: Optional[np.ndarray] = None
        self._stereo_baseline_m: Optional[float] = None
        self._extrinsics_ir_l2r: Optional[tuple] = None
        self._extrinsics_ir_r2l: Optional[tuple] = None
        self._laser_power: Optional[float] = laser_power
        self._exposure: Optional[float] = exposure

    def open(self) -> bool:
        self._pipe = rs.pipeline()
        self._cfg = rs.config()
        self._cfg.enable_stream(rs.stream.depth, self._depth_w, self._depth_h, rs.format.z16, self._fps)
        self._cfg.enable_stream(rs.stream.color, self._color_w, self._color_h, rs.format.bgr8, self._fps)
        if self._enable_ir:
            self._cfg.enable_stream(rs.stream.infrared, 1, self._depth_w, self._depth_h, rs.format.y8, self._fps)
            self._cfg.enable_stream(rs.stream.infrared, 2, self._depth_w, self._depth_h, rs.format.y8, self._fps)
        profile = self._pipe.start(self._cfg)
        dev = profile.get_device()
        depth_sensor = dev.first_depth_sensor()
        depth_sensor.set_option(rs.option.laser_power, float(cast(float, self._laser_power)))
        print("Laser Power Status:")
        print(depth_sensor.get_option(rs.option.laser_power))
        if self._exposure is not None:
            depth_sensor.set_option(rs.option.enable_auto_exposure, 0)
            depth_sensor.set_option(rs.option.exposure, float(self._exposure))
            print("Exposure Status:")
            print(depth_sensor.get_option(rs.option.exposure))
            print("Auto Exposure Status:")
            print(depth_sensor.get_option(rs.option.enable_auto_exposure))

    
        pipe_profile = self._pipe.get_active_profile()
        depth_sp = pipe_profile.get_stream(rs.stream.depth).as_video_stream_profile()
        color_sp = pipe_profile.get_stream(rs.stream.color).as_video_stream_profile()
        d_intr = depth_sp.get_intrinsics()
        c_intr = color_sp.get_intrinsics()
        self._rs_depth_intrinsics = d_intr
        self._rs_color_intrinsics = c_intr
        self._intrinsics = np.array([float(d_intr.fx), float(d_intr.fy), float(d_intr.ppx), float(d_intr.ppy)], dtype=np.float32)
        self._color_intrinsics = np.array([float(c_intr.fx), float(c_intr.fy), float(c_intr.ppx), float(c_intr.ppy)], dtype=np.float32)
        self._color_dist_coeffs = np.array(list(c_intr.coeffs), dtype=np.float32)
        ex = color_sp.get_extrinsics_to(depth_sp)
        R = np.array(ex.rotation, dtype=np.float32).reshape(3, 3)
        t_m = np.array(ex.translation, dtype=np.float32).reshape(3)
        t_mm = t_m * 1000.0
        self._extrinsics_c2d = (R, t_mm)
        ex_d = depth_sp.get_extrinsics_to(color_sp)
        R_d = np.array(ex_d.rotation, dtype=np.float32).reshape(3, 3)
        t_d_m = np.array(ex_d.translation, dtype=np.float32).reshape(3)
        t_d_mm = t_d_m * 1000.0
        self._extrinsics_d2c = (R_d, t_d_mm)
        if self._enable_ir:
            ir_left_sp = pipe_profile.get_stream(rs.stream.infrared, 1).as_video_stream_profile()
            ir_right_sp = pipe_profile.get_stream(rs.stream.infrared, 2).as_video_stream_profile()
            intr_l = ir_left_sp.get_intrinsics()
            intr_r = ir_right_sp.get_intrinsics()
            self._rs_ir_left_intrinsics = intr_l
            self._rs_ir_right_intrinsics = intr_r
            self._ir_left_intrinsics = np.array([float(intr_l.fx), float(intr_l.fy), float(intr_l.ppx), float(intr_l.ppy)], dtype=np.float32)
            self._ir_right_intrinsics = np.array([float(intr_r.fx), float(intr_r.fy), float(intr_r.ppx), float(intr_r.ppy)], dtype=np.float32)
            ex_l2r = ir_left_sp.get_extrinsics_to(ir_right_sp)
            R_l2r = np.array(ex_l2r.rotation, dtype=np.float32).reshape(3, 3)
            t_l2r_m = np.array(ex_l2r.translation, dtype=np.float32).reshape(3)
            t_l2r_mm = t_l2r_m * 1000.0
            self._extrinsics_ir_l2r = (R_l2r, t_l2r_mm)
            ex_r2l = ir_right_sp.get_extrinsics_to(ir_left_sp)
            R_r2l = np.array(ex_r2l.rotation, dtype=np.float32).reshape(3, 3)
            t_r2l_m = np.array(ex_r2l.translation, dtype=np.float32).reshape(3)
            t_r2l_mm = t_r2l_m * 1000.0
            self._extrinsics_ir_r2l = (R_r2l, t_r2l_mm)
            self._stereo_baseline_m = float(abs(t_l2r_m[0]))
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
            } if self._extrinsics_c2d else None,
            "extrinsics_depth_to_color": {
                "R": self._extrinsics_d2c[0].tolist(),
                "t": self._extrinsics_d2c[1].tolist()
            } if self._extrinsics_d2c else None,
            "extrinsics_ir_left_to_right": {
                "R": self._extrinsics_ir_l2r[0].tolist(),
                "t": self._extrinsics_ir_l2r[1].tolist()
            } if self._extrinsics_ir_l2r else None,
            "extrinsics_ir_right_to_left": {
                "R": self._extrinsics_ir_r2l[0].tolist(),
                "t": self._extrinsics_ir_r2l[1].tolist()
            } if self._extrinsics_ir_r2l else None
        }

    def get_rs_color_intrinsics(self) -> rs.intrinsics:
        return self._rs_color_intrinsics

    def get_rs_ir_left_intrinsics(self) -> rs.intrinsics:
        return self._rs_ir_left_intrinsics

    def read_frame(self) -> Optional[Frame]:
        if self._pipe is None:
            return None

        frames = self._pipe.wait_for_frames()
        ts = time.time()
        depth_frame = frames.get_depth_frame()
        if depth_frame is None:
            raise RuntimeError("failed to capture depth image")
        color_frame = frames.get_color_frame()
        ir_frame_left = None
        ir_frame_right = None
        if self._enable_ir:
            ir_frame_left = frames.get_infrared_frame(1)
            ir_frame_right = frames.get_infrared_frame(2)
        depth = np.asanyarray(depth_frame.get_data())
        if depth.dtype != np.uint16:
            depth = depth.astype(np.uint16)
        color = None
        if color_frame is not None:
            color = np.asanyarray(color_frame.get_data())
        ir_left = None
        ir_right = None
        if ir_frame_left is not None:
            ir_arr = np.asanyarray(ir_frame_left.get_data())
            if ir_arr.dtype == np.uint8:
                ir_left = (ir_arr.astype(np.uint16) * 257)
            elif ir_arr.dtype == np.uint16:
                ir_left = ir_arr
            else:
                ir_left = ir_arr.astype(np.uint16)
        if ir_frame_right is not None:
            ir_arr_r = np.asanyarray(ir_frame_right.get_data())
            if ir_arr_r.dtype == np.uint8:
                ir_right = (ir_arr_r.astype(np.uint16) * 257)
            elif ir_arr_r.dtype == np.uint16:
                ir_right = ir_arr_r
            else:
                ir_right = ir_arr_r.astype(np.uint16)
        if self._intrinsics is None:
            raise RuntimeError("intrinsics not loaded")
        frame = Frame(
            timestamp=ts,
            frame_id=self._frame_id,
            color=color,
            depth=depth,
            intrinsics=self._intrinsics,
            ir=ir_left,
            ir_main=ir_left,
            ir_aux=ir_right,
            ir_main_intrinsics=self._ir_left_intrinsics,
            ir_aux_intrinsics=self._ir_right_intrinsics,
            stereo_baseline_m=self._stereo_baseline_m,
            color_intrinsics=self._color_intrinsics,
            extrinsics_color_to_depth=self._extrinsics_c2d
        )
        self._frame_id += 1
        return frame

    def close(self):
        if self._pipe:
            self._pipe.stop()
            self._pipe = None
