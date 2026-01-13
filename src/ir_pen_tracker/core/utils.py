import numpy as np
import cv2

class CVUtils:
    @staticmethod
    def project_point(p3d, intrinsics, dist_coeffs=None, scale=1.0):
        # p3d: (N, 3) or (3,)
            
        fx, fy, cx, cy = [float(x) for x in intrinsics.tolist()]
        
        # Construct Camera Matrix
        # Note: If we use dist_coeffs, we should project on the original resolution 
        # and THEN scale the coordinates, because distortion is relative to the optical center 
        # and image size of the calibration.
        # Scaling intrinsics BEFORE projection with distortion is tricky/incorrect 
        # if distortion is non-trivial.
        # So: Project with original intrinsics -> Scale output UV.
        
        x, y, z = float(p3d[0]), float(p3d[1]), float(p3d[2])
        if z <= 0:
            return None
        u = int(x * fx / z + cx)
        v = int(y * fy / z + cy)
        
        # Apply scale to the resulting coordinates
        if scale != 1.0:
            u *= scale
            v *= scale
            
        return (int(u), int(v))

    @staticmethod
    def transform_point(p_source, extrinsics):
        """
        Generic rigid body transformation.
        P_target = R * P_source + t
        extrinsics: {'R': [...], 't': [...]}
        """
        R = np.array(extrinsics['R'])
        t = np.array(extrinsics['t']).flatten() / 1000.0 # Convert mm to meters
        
        # Ensure p_source is column vector or handle numpy broadcasting
        # p_source shape: (3,) or (N, 3)
        # R @ p_source + t
        
        return R @ p_source + t
    
    @staticmethod
    def depth_to_vis(depth_mm, d_min=200, d_max=2000):
        if depth_mm is None:
            return None
        arr = depth_mm.astype(np.uint16)
        mask = arr == 0
        arr = np.clip(arr, d_min, d_max)
        arr = ((arr - d_min) * 255.0 / max(1, (d_max - d_min))).astype(np.uint8)
        vis = cv2.applyColorMap(arr, cv2.COLORMAP_JET)
        vis[mask] = (0, 0, 0)
        return vis
    
    @staticmethod
    def ir_to_vis(ir_u16):
        if ir_u16 is None:
            return None
        x = ir_u16.astype(np.float32)
        valid = x[x > 0]
        if valid.size > 0:
            p95 = float(np.percentile(valid, 95))
            p95 = max(p95, 1.0)
            x = np.clip(x, 0.0, p95)
            x = (x / p95) * 255.0
        else:
            x = np.clip(x, 0.0, 1500.0) / 1500.0 * 255.0
        return x.astype(np.uint8)
