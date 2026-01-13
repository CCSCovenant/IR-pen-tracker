from dataclasses import dataclass
import numpy as np
from typing import Optional, List

@dataclass
class Frame:
    """Raw sensor data frame"""
    timestamp: float
    frame_id: int
    color: Optional[np.ndarray]          # HxWx3 BGR (Raw Color if using calibration)
    depth: np.ndarray          # HxW uint16 (mm)
    intrinsics: np.ndarray     # [fx, fy, cx, cy] (For the active camera - usually depth for tracking)
    ir: Optional[np.ndarray] = None   # HxW uint16
    ir_main: Optional[np.ndarray] = None
    ir_aux: Optional[np.ndarray] = None
    ir_main_intrinsics: Optional[np.ndarray] = None
    ir_aux_intrinsics: Optional[np.ndarray] = None
    stereo_baseline_m: Optional[float] = None
    
    # Optional calibration data for advanced users (e.g. Color to Depth mapping)
    color_intrinsics: Optional[np.ndarray] = None # [fx, fy, cx, cy]
    extrinsics_color_to_depth: Optional[tuple[np.ndarray, np.ndarray]] = None # 4x4 Transformation Matrix

@dataclass
class BrushPoseVis:
    """Vision-only brush tracking result"""
    timestamp: float
    tip_pos_cam: np.ndarray    # [x, y, z] in meters
    direction: np.ndarray      # Normalized vector
    quality: float
    has_lock: bool
    tail_pos_cam: Optional[np.ndarray] = None # [x, y, z] in meters

@dataclass
class BrushPose:
    """Final fused brush pose"""
    timestamp: float
    position: np.ndarray       # [x, y, z] in meters
    rotation: np.ndarray       # Quaternion [x, y, z, w]
    has_lock: bool

@dataclass
class Joint:
    name: str
    position: np.ndarray       # [x, y, z] in meters
    confidence: float

@dataclass
class Skeleton:
    joints: List[Joint]
    confidence: float
