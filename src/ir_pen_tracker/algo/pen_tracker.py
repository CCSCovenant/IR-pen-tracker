import numpy as np
import cv2
from dataclasses import dataclass
from typing import Optional, Tuple, List, Dict, Any
from ..core.interfaces import IBrushTracker
from ..core.types import Frame, BrushPoseVis

@dataclass
class IRPenConfig:
    # IR Threshold
    ir_threshold: int = 60000
    
    min_area: int = 10
    max_area: int = 2000
    
    # ROI & Sampling
    roi_scale: float = 0.5
    sample_margin: float = 0.2
    
    # Pen geometry
    pen_length_m: float = 0.15
    length_tolerance_m: float = 0.05
    
    # RANSAC
    ransac_threshold: float = 0.005 # 5mm
    
    min_depth_m: float = 0.1
    max_depth_m: float = 3.0

    @staticmethod
    def from_dict(config_dict: Dict[str, Any]) -> 'IRPenConfig':
        cfg = IRPenConfig()
        for k, v in config_dict.items():
            if hasattr(cfg, k):
                setattr(cfg, k, v)
        return cfg

def _get_blob_centroids_with_radius(mask: np.ndarray, min_area: int, max_area: int) -> List[Tuple[float, float, float]]:
    """Returns list of (u, v, radius)"""
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
    results = []
    for i in range(1, num_labels): # Skip background 0
        area = stats[i, cv2.CC_STAT_AREA]
        if min_area <= area <= max_area:
            # Approx radius from area: area = pi * r^2 -> r = sqrt(area / pi)
            r = np.sqrt(area / np.pi)
            results.append((centroids[i][0], centroids[i][1], r))
    return results

def _backproject(u: float, v: float, z_m: float, intrinsics: np.ndarray) -> np.ndarray:
    fx, fy, cx, cy = [float(x) for x in intrinsics.tolist()]
    x = (u - cx) * z_m / fx
    y = (v - cy) * z_m / fy
    return np.array([x, y, z_m], dtype=np.float32)

def _fit_line_ransac(points: np.ndarray, threshold: float = 0.005, max_iters: int = 100) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    """
    Fits a 3D line to points using RANSAC.
    points: (N, 3)
    Returns: (point_on_line, direction_vector) or None
    """
    n_points = points.shape[0]
    if n_points < 2:
        return None

    best_inliers = 0
    best_line = None # (p, d)
    
    # Pre-calculate to avoid loop overhead if possible, but pure python loop is slow.
    # Simple Python RANSAC
    
    rng = np.random.default_rng()
    
    for _ in range(max_iters):
        # Sample 2 points
        idx = rng.choice(n_points, 2, replace=False)
        p1 = points[idx[0]]
        p2 = points[idx[1]]
        
        direction = p2 - p1
        norm = np.linalg.norm(direction)
        if norm < 1e-6:
            continue
        direction /= norm
        
        # Distance from point P to line (A, d): || (P-A) x d ||
        # Vector from p1 to all points
        vecs = points - p1
        cross_prods = np.cross(vecs, direction)
        dists = np.linalg.norm(cross_prods, axis=1)
        
        inliers = np.sum(dists < threshold)
        
        if inliers > best_inliers:
            best_inliers = inliers
            best_line = (p1, direction)
            
    # Optional: Re-fit line using all inliers using PCA/SVD
    if best_line is not None and best_inliers > 2:
        p1_best, dir_best = best_line
        vecs = points - p1_best
        cross_prods = np.cross(vecs, dir_best)
        dists = np.linalg.norm(cross_prods, axis=1)
        inlier_mask = dists < threshold
        inlier_points = points[inlier_mask]
        
        if len(inlier_points) >= 2:
            # PCA for line fitting
            # 1. Centroid
            centroid = np.mean(inlier_points, axis=0)
            # 2. SVD
            uu, dd, vv = np.linalg.svd(inlier_points - centroid)
            direction = vv[0] # First principal component
            return (centroid, direction)
            
    return best_line

def _closest_point_on_line_from_ray(line_pt: np.ndarray, line_dir: np.ndarray, 
                                    ray_origin: np.ndarray, ray_dir: np.ndarray) -> np.ndarray:
    """
    Finds the point on line (line_pt, line_dir) that is closest to ray (ray_origin, ray_dir).
    This is effectively finding the closest points between two skew lines, and returning the one on the first line.
    
    L1(t) = P1 + t*d1  (Cylinder Axis)
    L2(s) = P2 + s*d2  (Camera Ray)
    
    We want to minimize || L1(t) - L2(s) ||^2.
    Let w0 = P1 - P2.
    a = d1.d1 = 1
    b = d1.d2
    c = d2.d2 = 1
    d = d1.w0
    e = d2.w0
    
    t = (b*e - c*d) / (a*c - b^2)
    s = (a*e - b*d) / (a*c - b^2)
    """
    p1 = line_pt
    d1 = line_dir
    p2 = ray_origin
    d2 = ray_dir
    
    w0 = p1 - p2
    a = 1.0 # np.dot(d1, d1)
    b = np.dot(d1, d2)
    c = 1.0 # np.dot(d2, d2)
    d = np.dot(d1, w0)
    e = np.dot(d2, w0)
    
    denom = a * c - b * b
    if abs(denom) < 1e-6:
        # Lines are parallel, return projection of p2 onto L1
        t = -d
    else:
        t = (b * e - c * d) / denom
        
    return p1 + t * d1

class IRPenTracker(IBrushTracker):
    def __init__(self, config: IRPenConfig = IRPenConfig()):
        self._cfg = config
        self._last_tip_pos: Optional[np.ndarray] = None

    def track_debug(self, frame: Frame) -> Tuple[BrushPoseVis, Dict[str, Any]]:
        debug_info: Dict[str, Any] = {
            "mask_ir": None,
            "blobs_uv": [],
            "blobs_radius": [],
            "roi_points_uv": [],
            "sample_points_3d": [], # Points used for RANSAC
            "fitted_line": None,    # (pt, dir)
            "final_tip": None,
            "final_tail": None,
            "roi_mask": None
        }

        if frame.ir is None:
            return BrushPoseVis(frame.timestamp, np.zeros(3), np.zeros(3), 0.0, False), debug_info
        
        # 1. Threshold IR (High Threshold)
        _, mask_ir = cv2.threshold(frame.ir, self._cfg.ir_threshold, 65535, cv2.THRESH_BINARY)
        mask_ir = mask_ir.astype(np.uint8)
        debug_info["mask_ir"] = mask_ir
        
        # 2. Find Blobs
        blobs = _get_blob_centroids_with_radius(mask_ir, self._cfg.min_area, self._cfg.max_area)
        # Sort by radius descending (assuming markers are the largest bright spots)
        blobs.sort(key=lambda x: x[2], reverse=True)
        
        if len(blobs) < 2:
            return BrushPoseVis(frame.timestamp, np.zeros(3), np.zeros(3), 0.0, False), debug_info
            
        # Take top 2 blobs
        b1 = blobs[0]
        b2 = blobs[1]
        
        u1, v1, r1 = b1
        u2, v2, r2 = b2
        
        debug_info["blobs_uv"] = [(u1, v1), (u2, v2)]
        debug_info["blobs_radius"] = [r1, r2]
        
        # 3. Define ROI & Sample Depth
        # Line segment length
        seg_len = np.hypot(u2 - u1, v2 - v1)
        if seg_len < 1.0:
            return BrushPoseVis(frame.timestamp, np.zeros(3), np.zeros(3), 0.0, False), debug_info

        # Direction vector in 2D
        du = (u2 - u1) / seg_len
        dv = (v2 - v1) / seg_len
        
        # Normal vector
        nu = -dv
        nv = du
        
        # Dynamic ROI width
        avg_r = (r1 + r2) / 2.0
        roi_w = avg_r * self._cfg.roi_scale
        debug_info["roi_width"] = roi_w
        
        # Sampling steps along the line
        # Avoid margin near markers
        margin = self._cfg.sample_margin * seg_len
        start_dist = margin
        end_dist = seg_len - margin
        
        if start_dist >= end_dist:
            return BrushPoseVis(frame.timestamp, np.zeros(3), np.zeros(3), 0.0, False), debug_info
             
        # Generate sample grid
        # We can iterate over bounding box of the line segment for simplicity, or rotate.
        # Let's simple iterate along the line and sample perpendicular
        
        # Store ROI corners for visualization
        roi_corners = [
            (u1 + nu * (-roi_w), v1 + nv * (-roi_w)),
            (u1 + nu * (roi_w), v1 + nv * (roi_w)),
            (u2 + nu * (roi_w), v2 + nv * (roi_w)),
            (u2 + nu * (-roi_w), v2 + nv * (-roi_w))
        ]
        debug_info["roi_corners"] = roi_corners

        sample_points_3d = []
        roi_points_uv = []
        
        # Sampling step size (e.g. 1 pixel)
        steps_l = int(end_dist - start_dist)
        steps_w = int(roi_w * 2) # from -roi_w to +roi_w
        
        if steps_l < 1:
            return BrushPoseVis(frame.timestamp, np.zeros(3), np.zeros(3), 0.0, False), debug_info
            
        # Create a mask for visualization
        h, w = frame.depth.shape
        roi_mask = np.zeros((h, w), dtype=np.uint8)

        # Vectorized sampling could be faster, but loop is easier to implement for rotated rect
        # Center of line: P(t) = P1 + t * D
        # Perpendicular: Q(t, s) = P(t) + s * N
        
        # Precompute intrinsics
        fx, fy, cx, cy = [float(x) for x in frame.intrinsics.tolist()]
        
        # Convert depth to meters
        depth_m = frame.depth.astype(np.float32) / 1000.0
        
        for t in np.linspace(start_dist, end_dist, steps_l):
            uc = u1 + du * t
            vc = v1 + dv * t
            
            for s in np.linspace(-roi_w, roi_w, max(1, steps_w)):
                u_s = int(round(uc + nu * s))
                v_s = int(round(vc + nv * s))
                
                if 0 <= u_s < w and 0 <= v_s < h:
                    roi_mask[v_s, u_s] = 255
                    roi_points_uv.append((u_s, v_s))
                    
                    z = depth_m[v_s, u_s]
                    if z > self._cfg.min_depth_m and z < self._cfg.max_depth_m:
                        # Backproject
                        x = (u_s - cx) * z / fx
                        y = (v_s - cy) * z / fy
                        sample_points_3d.append([x, y, z])

        debug_info["roi_mask"] = roi_mask
        debug_info["roi_points_uv"] = roi_points_uv
        
        pts_3d = np.array(sample_points_3d)
        debug_info["sample_points_3d"] = pts_3d
        
        if len(pts_3d) < 10: # Not enough points
            return BrushPoseVis(frame.timestamp, np.zeros(3), np.zeros(3), 0.0, False), debug_info
            
        # 4. RANSAC Fit
        line_res = _fit_line_ransac(pts_3d, self._cfg.ransac_threshold)
        if line_res is None:
            return BrushPoseVis(frame.timestamp, np.zeros(3), np.zeros(3), 0.0, False), debug_info
             
        line_pt, line_dir = line_res
        debug_info["fitted_line"] = (line_pt, line_dir)
        
        # 5. Project Markers onto 3D Line
        # Ray 1
        ray1_dir = np.array([(u1 - cx)/fx, (v1 - cy)/fy, 1.0])
        ray1_dir /= np.linalg.norm(ray1_dir)
        
        # Ray 2
        ray2_dir = np.array([(u2 - cx)/fx, (v2 - cy)/fy, 1.0])
        ray2_dir /= np.linalg.norm(ray2_dir)
        
        ray_origin = np.zeros(3) # Camera center
        
        p1_on_line = _closest_point_on_line_from_ray(line_pt, line_dir, ray_origin, ray1_dir)
        p2_on_line = _closest_point_on_line_from_ray(line_pt, line_dir, ray_origin, ray2_dir)
        
        # 6. Identify Tip vs Tail
        # Heuristic: Tip is "lower" in image (larger V) or closer to last tip
        tip_pos = p1_on_line
        tail_pos = p2_on_line
        
        if self._last_tip_pos is not None:
            d1 = np.linalg.norm(p1_on_line - self._last_tip_pos)
            d2 = np.linalg.norm(p2_on_line - self._last_tip_pos)
            if d2 < d1:
                tip_pos = p2_on_line
                tail_pos = p1_on_line
        else:
            if v2 > v1:
                tip_pos = p2_on_line
                tail_pos = p1_on_line

        self._last_tip_pos = tip_pos
        
        debug_info["final_tip"] = tip_pos
        debug_info["final_tail"] = tail_pos
        
        direction = tail_pos - tip_pos
        norm = np.linalg.norm(direction)
        if norm > 1e-6:
            direction /= norm
            
        res = BrushPoseVis(
            timestamp=frame.timestamp,
            tip_pos_cam=tip_pos,
            direction=direction,
            quality=1.0, 
            has_lock=True,
            tail_pos_cam=tail_pos
        )
        
        return res, debug_info

    def track(self, frame: Frame) -> BrushPoseVis:
        res, _ = self.track_debug(frame)
        return res
