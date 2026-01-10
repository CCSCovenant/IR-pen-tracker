import argparse
import os
from typing import Optional

import cv2

from kinect_pen.algo.body_tracker import MediaPipePoseBodyTracker, MediaPipePoseConfig
from kinect_pen.io.file_camera import FileCamera


_UPPER_BODY_EDGES = [
    ("left_shoulder", "right_shoulder"),
    ("left_shoulder", "left_elbow"),
    ("left_elbow", "left_wrist"),
    ("right_shoulder", "right_elbow"),
    ("right_elbow", "right_wrist"),
    ("left_shoulder", "left_hip"),
    ("right_shoulder", "right_hip"),
]


def _project(xyz_m, intrinsics):
    x, y, z = float(xyz_m[0]), float(xyz_m[1]), float(xyz_m[2])
    if not (z > 0.0):
        return None
    fx, fy, cx, cy = [float(v) for v in intrinsics.tolist()]
    u = x * fx / z + cx
    v = y * fy / z + cy
    return int(round(u)), int(round(v))


def _draw_skeleton(img_bgr, skeleton, intrinsics):
    if skeleton is None:
        return img_bgr
    out = img_bgr.copy()
    pts = {}
    for j in skeleton.joints:
        if j.position is None:
            continue
        if not (j.position[2] == j.position[2]):
            continue
        uv = _project(j.position, intrinsics)
        if uv is None:
            continue
        pts[j.name] = uv

    for name, uv in pts.items():
        cv2.circle(out, uv, 4, (0, 255, 0), -1)
        cv2.putText(out, name, (uv[0] + 4, uv[1] - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

    for a, b in _UPPER_BODY_EDGES:
        if a in pts and b in pts:
            cv2.line(out, pts[a], pts[b], (0, 255, 255), 2)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", required=True, help="raw dataset root containing color/depth/meta.json")
    ap.add_argument("--out", default=None, help="optional output video path")
    ap.add_argument("--max-frames", type=int, default=200)
    args = ap.parse_args()

    cam = FileCamera(args.data)
    cam.open()
    tracker = MediaPipePoseBodyTracker(MediaPipePoseConfig())

    writer: Optional[cv2.VideoWriter] = None
    out_path = args.out
    if out_path:
        os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)

    while True:
        frame = cam.read_frame()
        if frame is None:
            break
        skeleton = tracker.track(frame)
        vis = _draw_skeleton(frame.color, skeleton, frame.intrinsics)
        if out_path and writer is None:
            h, w = vis.shape[:2]
            writer = cv2.VideoWriter(out_path, cv2.VideoWriter_fourcc(*"mp4v"), 5.0, (w, h))
        if writer is not None:
            writer.write(vis)
        if frame.frame_id + 1 >= int(args.max_frames):
            break

    cam.close()
    if writer is not None:
        writer.release()


if __name__ == "__main__":
    main()
