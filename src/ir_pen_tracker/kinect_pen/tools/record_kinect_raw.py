import os
import json
import time
import cv2
from datetime import datetime

from kinect_pen.io.kinect_camera import KinectCamera

def main():
    out_root = os.path.join(os.getcwd(), "data", "raw_" + datetime.now().strftime("%Y%m%d_%H%M%S"))
    color_dir = os.path.join(out_root, "color")
    depth_dir = os.path.join(out_root, "depth")
    os.makedirs(color_dir, exist_ok=True)
    os.makedirs(depth_dir, exist_ok=True)

    cam = KinectCamera()
    cam.open()

    fps = 5.0
    timestamps = []
    intrinsics = None
    n = 100
    for i in range(n):
        f = cam.read_frame()
        if intrinsics is None:
            intrinsics = {
                "fx": float(f.intrinsics[0]),
                "fy": float(f.intrinsics[1]),
                "cx": float(f.intrinsics[2]),
                "cy": float(f.intrinsics[3]),
            }
        timestamps.append(float(f.timestamp))
        color_path = os.path.join(color_dir, f"{i:06d}.png")
        depth_path = os.path.join(depth_dir, f"{i:06d}.png")
        cv2.imwrite(color_path, f.color)
        d = f.depth
        if d.dtype != "uint16":
            d = d.astype("uint16")
        cv2.imwrite(depth_path, d)
        time.sleep(max(0.0, (1.0 / fps)))

    cam.close()

    meta = {
        "intrinsics": intrinsics,
        "fps": fps,
        "timestamps": timestamps,
    }
    with open(os.path.join(out_root, "meta.json"), "w", encoding="utf-8") as fw:
        json.dump(meta, fw, ensure_ascii=False, indent=2)

    print(out_root)

if __name__ == "__main__":
    main()
