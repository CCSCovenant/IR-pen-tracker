import os
import time
import json
import cv2
from datetime import datetime
from PyQt5.QtCore import QObject, pyqtSignal

class RecordingManager(QObject):
    status_update = pyqtSignal(str)

    def __init__(self, project_root):
        super().__init__()
        self.project_root = project_root
        self.is_recording = False
        self.rec_dir = None
        self.rec_out_color = None
        self.rec_file_pen = None
        self.rec_frame_idx = 0
        self.rec_start_time = 0
        
    def start(self, cam, desk_plane_depth):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.rec_dir = os.path.join(self.project_root, "recordings", f"rec_{timestamp}")
        
        os.makedirs(self.rec_dir, exist_ok=True)
        os.makedirs(os.path.join(self.rec_dir, "depth"), exist_ok=True)
        os.makedirs(os.path.join(self.rec_dir, "ir"), exist_ok=True)
        
        # Save Metadata
        meta = cam.get_calibration_data()
        meta["desk_plane"] = desk_plane_depth.tolist() if desk_plane_depth is not None else None
        with open(os.path.join(self.rec_dir, "meta.json"), 'w') as f:
            json.dump(meta, f, indent=2)
            
        self.rec_out_color = cv2.VideoWriter(
            os.path.join(self.rec_dir, "color.avi"),
            cv2.VideoWriter_fourcc(*'MJPG'),
            30.0,
            (1920, 1080)
        )
        
        self.rec_file_pen = open(os.path.join(self.rec_dir, "pen_data.jsonl"), 'w')
        self.rec_frame_idx = 0
        self.rec_start_time = time.time()
        self.is_recording = True
        self.status_update.emit(f"Recording started: {os.path.basename(self.rec_dir)}")

    def stop(self):
        if not self.is_recording:
            return
            
        self.is_recording = False
        if self.rec_out_color:
            self.rec_out_color.release()
            self.rec_out_color = None
        if self.rec_file_pen:
            self.rec_file_pen.close()
            self.rec_file_pen = None
            
        self.status_update.emit(f"Recording saved: {os.path.basename(self.rec_dir)}")
        self.rec_dir = None

    def record_frame(self, frame, tracker_result):
        if not self.is_recording:
            return
            
        # 1. Color Video
        if frame.color is not None:
            c_img = frame.color
            if len(c_img.shape) == 3 and c_img.shape[2] == 4:
                c_img = cv2.cvtColor(c_img, cv2.COLOR_BGRA2BGR)
            self.rec_out_color.write(c_img)
            
        # 2. Raw Depth/IR
        if frame.depth is not None:
            cv2.imwrite(os.path.join(self.rec_dir, "depth", f"{self.rec_frame_idx:06d}.png"), 
                        frame.depth, [cv2.IMWRITE_PNG_COMPRESSION, 0])
        if frame.ir is not None:
            cv2.imwrite(os.path.join(self.rec_dir, "ir", f"{self.rec_frame_idx:06d}.png"), 
                        frame.ir, [cv2.IMWRITE_PNG_COMPRESSION, 0])
            
        # 3. Pen Data
        pen_entry = {
            "frame_id": self.rec_frame_idx,
            "timestamp": frame.timestamp,
            "has_lock": tracker_result.has_lock,
            "tip": tracker_result.tip_pos_cam.tolist() if tracker_result.tip_pos_cam is not None else None,
            "direction": tracker_result.direction.tolist() if tracker_result.direction is not None else None,
        }
        self.rec_file_pen.write(json.dumps(pen_entry) + "\n")
        
        self.rec_frame_idx += 1
