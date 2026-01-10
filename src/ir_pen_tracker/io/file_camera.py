import os
import json
from typing import Optional, List
import numpy as np
import cv2

from ..core.interfaces import ICamera
from ..core.types import Frame

class FileCamera(ICamera):
    def __init__(self, root_dir: str, color_dir: str = "color", depth_dir: str = "depth", ir_dir: Optional[str] = None):
        self.root_dir = root_dir
        self.color_dir = os.path.join(root_dir, color_dir)
        self.depth_dir = os.path.join(root_dir, depth_dir)
        self.ir_dir = os.path.join(root_dir, ir_dir) if ir_dir else None
        self.meta_path = os.path.join(root_dir, "meta.json")
        self._pairs: List[tuple] = []
        self._intrinsics: Optional[np.ndarray] = None
        self._timestamps: Optional[List[float]] = None
        self._fps: Optional[float] = None
        self._frame_id = 0
        self._opened = False

    def open(self) -> bool:
        if not os.path.isdir(self.color_dir):
            raise FileNotFoundError(self.color_dir)
        if not os.path.isdir(self.depth_dir):
            raise FileNotFoundError(self.depth_dir)
        color_files = sorted([f for f in os.listdir(self.color_dir) if f.lower().endswith((".png", ".jpg", ".jpeg"))])
        depth_files = sorted([f for f in os.listdir(self.depth_dir) if f.lower().endswith(".png")])
        if len(color_files) == 0 or len(depth_files) == 0:
            raise RuntimeError("empty dataset")
        color_stems = {os.path.splitext(f)[0]: f for f in color_files}
        depth_stems = {os.path.splitext(f)[0]: f for f in depth_files}
        common = sorted(set(color_stems.keys()) & set(depth_stems.keys()))
        if len(common) > 0:
            self._pairs = [(os.path.join(self.color_dir, color_stems[s]), os.path.join(self.depth_dir, depth_stems[s])) for s in common]
        else:
            n = min(len(color_files), len(depth_files))
            self._pairs = [(os.path.join(self.color_dir, color_files[i]), os.path.join(self.depth_dir, depth_files[i])) for i in range(n)]
        if os.path.isfile(self.meta_path):
            with open(self.meta_path, "r", encoding="utf-8") as f:
                meta = json.load(f)
            intr = meta.get("intrinsics")
            if intr is None:
                raise ValueError("missing intrinsics")
            self._intrinsics = np.array([intr["fx"], intr["fy"], intr["cx"], intr["cy"]], dtype=np.float32)
            self._timestamps = meta.get("timestamps")
            self._fps = meta.get("fps")
        else:
            raise FileNotFoundError(self.meta_path)
        self._frame_id = 0
        self._opened = True
        return True

    def read_frame(self) -> Optional[Frame]:
        if not self._opened:
            return None
        if self._frame_id >= len(self._pairs):
            return None
        color_path, depth_path = self._pairs[self._frame_id]
        color = cv2.imread(color_path, cv2.IMREAD_COLOR)
        depth = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)
        if color is None or depth is None:
            raise RuntimeError("failed to read image")
        ir = None
        if self.ir_dir:
            stem = os.path.splitext(os.path.basename(color_path))[0]
            candidates = [f for f in os.listdir(self.ir_dir) if os.path.splitext(f)[0] == stem]
            if candidates:
                ir_path = os.path.join(self.ir_dir, candidates[0])
                ir = cv2.imread(ir_path, cv2.IMREAD_UNCHANGED)
        if self._timestamps and self._frame_id < len(self._timestamps):
            ts = float(self._timestamps[self._frame_id])
        else:
            if not self._fps or self._fps <= 0:
                raise ValueError("invalid fps")
            ts = self._frame_id / float(self._fps)
        if self._intrinsics is None:
            raise RuntimeError("intrinsics not loaded")
        frame = Frame(
            timestamp=ts,
            frame_id=self._frame_id,
            color=color,
            depth=depth,
            ir=ir,
            intrinsics=self._intrinsics,
        )
        self._frame_id += 1
        return frame

    def close(self):
        self._opened = False
        self._pairs = []
