import argparse
import time
from typing import Optional

import cv2
import numpy as np

from ir_pen_tracker.algo.body_tracker import MediaPipePoseBodyTracker, MediaPipePoseConfig
from ir_pen_tracker.algo.pen_tracker import ColorPenTracker, ColorPenConfig
from ir_pen_tracker.core.types import Frame
from ir_pen_tracker.io.kinect_camera import KinectCamera


class OpenCVCamera:
    """Fallback camera using standard OpenCV VideoCapture (no depth)."""

    def __init__(self, camera_id: int):
        self._cap = cv2.VideoCapture(camera_id)
        # 尝试设置与Kinect相似的分辨率
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self._intrinsics = None
        self._frame_id = 0

    def open(self) -> bool:
        return self._cap.isOpened()

    def read_frame(self) -> Optional[Frame]:
        if not self._cap.isOpened():
            return None
        ret, frame_bgr = self._cap.read()
        if not ret or frame_bgr is None:
            return None
        
        h, w = frame_bgr.shape[:2]
        
        # 伪造内参 (假设FOV ~60度)
        if self._intrinsics is None:
            fx = w  # 粗略估计
            fy = w
            cx = w / 2
            cy = h / 2
            self._intrinsics = np.array([fx, fy, cx, cy], dtype=np.float32)

        # 伪造深度图 (全0，表示无深度)
        # 使用16位整数表示毫米
        dummy_depth = np.zeros((h, w), dtype=np.uint16)

        ts = time.time()
        frame = Frame(
            timestamp=ts,
            frame_id=self._frame_id,
            color=frame_bgr,
            depth=dummy_depth,
            ir=None,
            intrinsics=self._intrinsics,
        )
        self._frame_id += 1
        return frame

    def close(self):
        self._cap.release()


def _draw_text_overlay(img, lines, pos=(20, 30), color=(0, 255, 0), thickness=2):
    """Draws multi-line text overlay."""
    x, y = pos
    for line in lines:
        cv2.putText(img, line, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), thickness + 2)
        cv2.putText(img, line, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, thickness)
        y += 25


def _draw_skeleton_and_overlay(img_bgr, skeleton, intrinsics):
    if skeleton is None:
        return img_bgr
    
    out = img_bgr.copy()
    h, w = out.shape[:2]
    
    # Check Tracking Source (1.0 = Pose, 2.0 = Hands)
    source_val = 0.0
    for j in skeleton.joints:
        if j.name == "tracking_source":
            source_val = j.confidence # Stored in conf channel for convenience in track_rgb, but here in skeleton it's usually stripped or handled differently.
            # Wait, track() implementation in body_tracker maps track_rgb output to Skeleton.
            # Let's check if 'tracking_source' survives into Skeleton.joints
            # The current body_tracker.py loop filters names? No, it just iterates.
            # But backproject logic might filter invalid 2D points.
            # Let's assume we can find it if we added it to output.
            pass
            
    # Draw connections
    # Standard MediaPipe Pose connections
    mp_connections = [
        # Torso
        ("left_shoulder", "right_shoulder"),
        ("left_shoulder", "left_hip"),
        ("right_shoulder", "right_hip"),
        ("left_hip", "right_hip"),
        # Arms
        ("left_shoulder", "left_elbow"),
        ("left_elbow", "left_wrist"),
        ("right_shoulder", "right_elbow"),
        ("right_elbow", "right_wrist"),
        # Hands (optional)
        ("left_wrist", "left_thumb"),
        ("left_wrist", "left_pinky"),
        ("left_wrist", "left_index"),
        ("right_wrist", "right_thumb"),
        ("right_wrist", "right_pinky"),
        ("right_wrist", "right_index"),
        
        # Extrapolated Arm Connections (for Hands mode)
        ("left_wrist", "left_elbow"), # Overlaps with above, that's fine
        ("right_wrist", "right_elbow"),
    ]
    
    # Create map for fast lookup
    j_map = {j.name: j for j in skeleton.joints}
    
    # Identify Source
    mode_text = "MODE: UNKNOWN"
    mode_color = (0, 255, 255) # Yellow
    
    if "tracking_source" in j_map:
        src = j_map["tracking_source"].confidence # We stored value in confidence slot
        # Actually in body_tracker.py track() method:
        # It calls _backproject. If 'tracking_source' is passed, it has u=0, v=0.
        # It will be backprojected to 3D or NaN.
        # But wait, track_rgb returns (name, u, v, conf).
        # track() iterates this list.
        # It calls _median_depth_m(depth, u, v...).
        # For 'tracking_source' (0,0), it might get valid depth or NaN.
        # This is a bit hacky. Let's rely on checking if 'nose' exists for Pose mode.
        pass
        
    if "nose" in j_map and not np.isnan(j_map["nose"].position[0]):
        mode_text = "MODE: FULL BODY (Pose)"
        mode_color = (0, 255, 0) # Green
    elif "left_wrist" in j_map or "right_wrist" in j_map:
         # If no nose but we have wrist, likely Hands mode (or partial Pose)
         # Check if we have 'tracking_source' marker from my previous edit
         # In previous edit: out.append(("tracking_source", 0.0, 0.0, 2.0))
         # In track(): u=0, v=0 -> _median_depth_m(0,0) -> likely valid or NaN.
         # But the name "tracking_source" will be in the joint list.
         if "tracking_source" in j_map:
             # We can try to read the "confidence" field from the joint, 
             # but track() converts conf to 1.0 usually? 
             # No, track() in body_tracker.py:
             # conf3d = conf (from track_rgb) * (valid_pixels / total_pixels) ... wait, no.
             # Let's look at body_tracker.py again.
             pass
         
         # Fallback logic: If we have wrist but no shoulder/nose, it's effectively Arm-Only
         if "left_shoulder" not in j_map and "right_shoulder" not in j_map:
             mode_text = "MODE: ARM ONLY (Hands Fallback)"
             mode_color = (0, 165, 255) # Orange
             
             # Add warning text
             cv2.putText(out, "NO BODY CONTEXT - ARM EXTRAPOLATED", (20, 100), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

    for a, b in mp_connections:
        if a in j_map and b in j_map:
            ja = j_map[a]
            jb = j_map[b]
            
            # Check if 2D coords are valid (we might have stored them in Skeleton if we extended it, 
            # but standard Skeleton has 3D pos. We need to project back or just use the 2D overlay from track_rgb?
            # run_live_vis.py currently gets Skeleton.
            # We don't have 2D coords in Skeleton easily unless we project back.
            # But wait! run_live_vis.py line 258: landmarks_2d = tracker.track_rgb(frame.color)
            # It calls track_rgb SEPARATELY for 2D visualization!
            # So this function _draw_skeleton_and_overlay is for 3D?
            # No, looking at usages...
            pass
            
    # ... (Rest of function logic is complex, let's just handle the text overlay part and let the loop handle lines)
    
    _draw_text_overlay(out, [mode_text], pos=(20, 70), color=mode_color)
    
    for a, b in mp_connections:
        if a in j_map and b in j_map:
             pa = j_map[a].position
             pb = j_map[b].position
             
             # Project to 2D
             # x = (u - cx) * z / fx  => u = x * fx / z + cx
             if np.isnan(pa).any() or np.isnan(pb).any():
                 continue
                 
             if pa[2] <= 0 or pb[2] <= 0:
                 continue

             fx, fy, cx, cy = intrinsics
             
             ua = int(pa[0] * fx / pa[2] + cx)
             va = int(pa[1] * fy / pa[2] + cy)
             ub = int(pb[0] * fx / pb[2] + cx)
             vb = int(pb[1] * fy / pb[2] + cy)
             
             cv2.line(out, (ua, va), (ub, vb), (0, 255, 0), 2)
             cv2.circle(out, (ua, va), 4, (0, 0, 255), -1)
             cv2.circle(out, (ub, vb), 4, (0, 0, 255), -1)

    return out


def _draw_pen_overlay(img_bgr, brush_pose, intrinsics):
    if brush_pose is None or not brush_pose.has_lock:
        return img_bgr

    out = img_bgr.copy()
    h, w = out.shape[:2]
    
    # Project Tip
    tip_3d = brush_pose.tip_pos_cam
    if tip_3d[2] > 0:
        fx, fy, cx, cy = intrinsics
        u = int(tip_3d[0] * fx / tip_3d[2] + cx)
        v = int(tip_3d[1] * fy / tip_3d[2] + cy)
        
        # Draw Tip
        cv2.circle(out, (u, v), 8, (0, 0, 255), -1) # Red Tip
        cv2.circle(out, (u, v), 12, (0, 255, 255), 2)
        cv2.putText(out, f"Tip Z:{tip_3d[2]:.2f}m", (u+15, v), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        # Draw Direction
        # End point = Tip - Direction * Length (e.g. 0.2m)
        # Note: Direction is vector from Tip to End? No, Spec says End - Tip.
        # Wait, my implementation: direction = p_end - p_tip. So it points FROM Tip TO End.
        # So to draw the "stick", we draw from Tip to Tip + Direction * 0.2
        end_3d = tip_3d + brush_pose.direction * 0.2
        
        ue = int(end_3d[0] * fx / end_3d[2] + cx)
        ve = int(end_3d[1] * fy / end_3d[2] + cy)
        
        cv2.line(out, (u, v), (ue, ve), (255, 0, 0), 3) # Blue shaft
        cv2.circle(out, (ue, ve), 5, (255, 0, 0), -1)

    return out


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera", type=int, default=0, help="Camera ID for fallback (default: 0)")
    args = parser.parse_args()

    # 1. Initialize Camera
    print("Initializing Camera...")
    try:
        camera = KinectCamera()
        if not camera.open():
            print("Failed to open Kinect. Falling back to OpenCV.")
            camera = OpenCVCamera(args.camera)
    except Exception as e:
        print(f"Error opening Kinect: {e}")
        camera = OpenCVCamera(args.camera)

    if not camera.open():
        print("Failed to open any camera.")
        return

    # 2. Initialize Trackers
    print("Initializing Body Tracker...")
    body_config = MediaPipePoseConfig()
    body_tracker = MediaPipePoseBodyTracker(body_config)
    
    print("Initializing Pen Tracker (RGB Mode)...")
    pen_config = ColorPenConfig()
    pen_tracker = ColorPenTracker(pen_config)

    print("Starting loop. Press 'q' to quit.")
    
    try:
        while True:
            # Read Frame
            frame = camera.read_frame()
            if frame is None:
                continue

            # Process
            t0 = time.time()
            skeleton = body_tracker.track(frame)
            brush_pose, debug_info = pen_tracker.track_debug(frame)
            dt = time.time() - t0
            fps = 1.0 / dt if dt > 0 else 0

            # Visualize RGB
            vis = frame.color.copy()
            
            # Blend Masks for Debug
            mask_red = debug_info.get("mask_red")
            if mask_red is not None:
                # Add Red Mask overlay (semi-transparent red)
                red_overlay = np.zeros_like(vis)
                red_overlay[mask_red > 0] = [0, 0, 255] # BGR: Red
                vis = cv2.addWeighted(vis, 1.0, red_overlay, 0.5, 0)
                
            mask_blue = debug_info.get("mask_blue")
            if mask_blue is not None:
                # Add Blue Mask overlay (semi-transparent blue)
                blue_overlay = np.zeros_like(vis)
                blue_overlay[mask_blue > 0] = [255, 0, 0] # BGR: Blue
                vis = cv2.addWeighted(vis, 1.0, blue_overlay, 0.5, 0)

            # Draw Centroids and Depth Info
            c_red = debug_info.get("c_red")
            z_tip = debug_info.get("z_tip")
            if c_red:
                cx, cy = int(c_red[0]), int(c_red[1])
                cv2.circle(vis, (cx, cy), 8, (0, 255, 255), -1) # Yellow Dot
                depth_str = f"{z_tip:.2f}m" if z_tip is not None else "NaN"
                cv2.putText(vis, f"Tip:{depth_str}", (cx + 10, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            c_blue = debug_info.get("c_blue")
            z_end = debug_info.get("z_end")
            if c_blue:
                cx, cy = int(c_blue[0]), int(c_blue[1])
                cv2.circle(vis, (cx, cy), 8, (255, 255, 0), -1) # Cyan Dot
                depth_str = f"{z_end:.2f}m" if z_end is not None else "NaN"
                cv2.putText(vis, f"End:{depth_str}", (cx + 10, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            
            # Draw Skeleton
            vis = _draw_skeleton_and_overlay(vis, skeleton, frame.intrinsics)
            
            # Draw Pen
            vis = _draw_pen_overlay(vis, brush_pose, frame.intrinsics)

            # Draw FPS
            cv2.putText(vis, f"FPS: {fps:.1f}", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            cv2.imshow("Kinect Pen Tracking (RGB)", vis)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        pass
    finally:
        print("Closing...")
        camera.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
