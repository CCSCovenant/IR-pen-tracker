from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, cast

import cv2
import mediapipe as mp
import numpy as np

from ..core.interfaces import IBodyTracker
from ..core.types import Frame, Joint, Skeleton


@dataclass(frozen=True)
class MediaPipePoseConfig:
    model_complexity: int = 1
    smooth_landmarks: bool = True
    enable_segmentation: bool = False
    smooth_segmentation: bool = False
    min_detection_confidence: float = 0.5
    min_tracking_confidence: float = 0.5
    depth_window: int = 5
    min_depth_m: float = 0.15
    max_depth_m: float = 6.0


def _median_depth_m(depth_mm: np.ndarray, u: int, v: int, win: int, min_m: float, max_m: float) -> float:
    h, w = depth_mm.shape[:2]
    r = win // 2
    x0 = max(0, u - r)
    x1 = min(w, u + r + 1)
    y0 = max(0, v - r)
    y1 = min(h, v + r + 1)
    patch = depth_mm[y0:y1, x0:x1].astype(np.float32) / 1000.0
    valid = patch[(patch > min_m) & (patch < max_m)]
    if valid.size == 0:
        return float("nan")
    return float(np.median(valid))


def _backproject(u: float, v: float, z_m: float, intrinsics: np.ndarray) -> np.ndarray:
    fx, fy, cx, cy = [float(x) for x in intrinsics.tolist()]
    x = (u - cx) * z_m / fx
    y = (v - cy) * z_m / fy
    return np.array([x, y, z_m], dtype=np.float32)


def _mp_landmark_name(lm: mp.solutions.pose.PoseLandmark) -> str:
    return lm.name.lower()


class MediaPipePoseBodyTracker(IBodyTracker):
    def __init__(self, config: MediaPipePoseConfig):
        self._cfg = config
        self._pose = mp.solutions.pose.Pose(
            static_image_mode=False,
            model_complexity=int(config.model_complexity),
            smooth_landmarks=bool(config.smooth_landmarks),
            enable_segmentation=bool(config.enable_segmentation),
            smooth_segmentation=bool(config.smooth_segmentation),
            min_detection_confidence=float(config.min_detection_confidence),
            min_tracking_confidence=float(config.min_tracking_confidence),
        )
        self._hands = mp.solutions.hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            model_complexity=1,
            min_detection_confidence=float(config.min_detection_confidence),
            min_tracking_confidence=float(config.min_tracking_confidence),
        )

    def track_rgb(self, color_bgr: np.ndarray) -> Optional[List[tuple]]:
        h, w = color_bgr.shape[:2]
        rgb = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2RGB)
        
        # 1. Try Pose
        result_pose = self._pose.process(rgb)
        pose_valid = False
        
        out: List[tuple] = []
        
        if result_pose.pose_landmarks is not None:
            # Check confidence of shoulders or nose to determine if we have a good body context
            # Indices: Nose=0, Left Shoulder=11, Right Shoulder=12
            landmarks = result_pose.pose_landmarks.landmark
            nose_conf = landmarks[mp.solutions.pose.PoseLandmark.NOSE].visibility
            l_shoulder_conf = landmarks[mp.solutions.pose.PoseLandmark.LEFT_SHOULDER].visibility
            r_shoulder_conf = landmarks[mp.solutions.pose.PoseLandmark.RIGHT_SHOULDER].visibility
            
            # Heuristic: If we see nose OR shoulders with decent confidence, we trust Pose
            if nose_conf > 0.5 or l_shoulder_conf > 0.5 or r_shoulder_conf > 0.5:
                pose_valid = True
                
                for idx, lm in enumerate(landmarks):
                    if idx >= len(mp.solutions.pose.PoseLandmark):
                        name = f"kp_{idx}"
                    else:
                        name = _mp_landmark_name(mp.solutions.pose.PoseLandmark(idx))
                    u = float(lm.x) * float(w)
                    v = float(lm.y) * float(h)
                    conf = float(getattr(lm, "visibility", 0.0) or 0.0)
                    out.append((name, u, v, conf))
                
                # Tag status
                out.append(("tracking_source", 0.0, 0.0, 1.0)) # 1.0 = Pose
                return out

        # 2. Fallback to Hands
        # If pose failed or confidence too low (only arm visible)
        if not pose_valid:
            result_hands = self._hands.process(rgb)
            if result_hands.multi_hand_landmarks:
                # Use the first hand found
                hand_landmarks = result_hands.multi_hand_landmarks[0]
                
                # Extract Wrist
                wrist = hand_landmarks.landmark[mp.solutions.hands.HandLandmark.WRIST]
                wrist_u, wrist_v = wrist.x * w, wrist.y * h
                
                # Extract Middle Finger MCP for direction
                middle = hand_landmarks.landmark[mp.solutions.hands.HandLandmark.MIDDLE_FINGER_MCP]
                middle_u, middle_v = middle.x * w, middle.y * h
                
                # Extrapolate Elbow
                # Vector from Middle -> Wrist
                vec_u = wrist_u - middle_u
                vec_v = wrist_v - middle_v
                
                # Extrapolate factor (e.g. 2.5x palm length)
                elbow_u = wrist_u + vec_u * 2.5
                elbow_v = wrist_v + vec_v * 2.5
                
                # Add to output
                # We map to standard names so downstream doesn't break
                # Note: We don't know if it's left or right hand easily without handedness check
                # For simplicity, we can try to guess or just output generic names.
                # However, existing code expects 'left_wrist', 'right_wrist' etc.
                # Let's check handedness
                handedness = result_hands.multi_handedness[0].classification[0].label # "Left" or "Right"
                prefix = handedness.lower() # "left" or "right"
                
                out.append((f"{prefix}_wrist", wrist_u, wrist_v, 0.9))
                out.append((f"{prefix}_elbow", elbow_u, elbow_v, 0.5)) # Low confidence as it's estimated
                
                # Add hand tip for visual confirmation?
                # Maybe not needed for skeleton but good for debug
                out.append((f"{prefix}_index", 
                            hand_landmarks.landmark[mp.solutions.hands.HandLandmark.INDEX_FINGER_TIP].x * w,
                            hand_landmarks.landmark[mp.solutions.hands.HandLandmark.INDEX_FINGER_TIP].y * h, 
                            0.9))

                out.append(("tracking_source", 0.0, 0.0, 2.0)) # 2.0 = Hands
                return out

        return None


    def track(self, frame: Frame) -> Optional[Skeleton]:
        landmarks = self.track_rgb(cast(np.ndarray, frame.color))
        if landmarks is None:
            return None

        joints: List[Joint] = []
        for name, u, v, conf in landmarks:
            ui = int(round(u))
            vi = int(round(v))
            if ui < 0 or vi < 0 or ui >= frame.depth.shape[1] or vi >= frame.depth.shape[0]:
                pos = np.array([np.nan, np.nan, np.nan], dtype=np.float32)
                conf3d = 0.0
            else:
                z = _median_depth_m(
                    frame.depth,
                    ui,
                    vi,
                    int(self._cfg.depth_window),
                    float(self._cfg.min_depth_m),
                    float(self._cfg.max_depth_m),
                )
                if not np.isfinite(z):
                    pos = np.array([np.nan, np.nan, np.nan], dtype=np.float32)
                    conf3d = 0.0
                else:
                    pos = _backproject(u, v, z, frame.intrinsics)
                    conf3d = conf

            joints.append(Joint(name=name, position=pos, confidence=conf3d))

        name_to_idx = {j.name: i for i, j in enumerate(joints)}
        if "left_wrist" in name_to_idx:
            j = joints[name_to_idx["left_wrist"]]
            joints.append(Joint(name="left_hand", position=j.position.copy(), confidence=float(j.confidence)))
        if "right_wrist" in name_to_idx:
            j = joints[name_to_idx["right_wrist"]]
            joints.append(Joint(name="right_hand", position=j.position.copy(), confidence=float(j.confidence)))

        joint_confs = np.array([j.confidence for j in joints], dtype=np.float32)
        sk_conf = float(np.nanmean(joint_confs)) if joint_confs.size > 0 else 0.0
        return Skeleton(joints=joints, confidence=sk_conf)
