import os
import time
import pyk4a
from PyQt5.QtCore import QThread, pyqtSignal

from ir_pen_tracker.io.kinect_camera import KinectCamera
from ir_pen_tracker.algo.pen_tracker import IRPenTracker, IRPenConfig
from ir_pen_tracker.algo.calibration import DeskCalibration
from ir_pen_tracker.vis.ortho_vis import OrthoVisualizer
from ir_pen_tracker.core.config_loader import load_config

from ir_pen_tracker.logic.managers.recording_manager import RecordingManager
from ir_pen_tracker.logic.processors.calibration_processor import CalibrationProcessor
from ir_pen_tracker.logic.processors.tracking_processor import TrackingProcessor

current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(current_dir, "..", "..", ".."))

class AppController(QThread):
    # Signals
    # frames is a dict: {'color': np.ndarray, 'ir': np.ndarray, 'ortho': np.ndarray}
    frames_ready = pyqtSignal(dict) 
    status_update = pyqtSignal(str)
    
    def __init__(self):
        super().__init__()
        self.config = load_config()
        print(f"Loaded config: {self.config.keys()}")
        
        self.running = True
        self.state = "INIT" # INIT, CALIBRATE, MAIN
        
        # Core Components
        self.cam = None
        self.tracker = None
        self.calib = DeskCalibration()
        self.ortho_vis = OrthoVisualizer()
        
        # Managers & Processors
        self.rec_manager = RecordingManager(project_root)
        self.rec_manager.status_update.connect(self.handle_status_update)
        
        self.calib_processor = None
        self.track_processor = None
        
        self.camera_type = str(self.config.get("camera", {}).get("type", "kinect")).lower()
        self.debug_skip_calibration = bool(self.config.get("debug", {}).get("skip_calibration", False))
        
        # State
        self.desk_plane_depth = None
        self.R_w = None
        self.origin_w = None
        
    def handle_status_update(self, msg):
        self.status_update.emit(msg)
        
    def initialize(self):
        self.status_update.emit("Initializing Camera...")
        self.initialize_camera()
        
        pen_cfg = IRPenConfig.from_dict(self.config.get("pen", {}))
        self.tracker = IRPenTracker(pen_cfg)
        
        # Init Processors
        self.calib_processor = CalibrationProcessor(self.calib, self.config)
        
        lowest_marker_to_tip_m = float(self.config.get("pen", {}).get("lowest_marker_to_tip_m", 0.13))
        self.track_processor = TrackingProcessor(self.config, self.ortho_vis, lowest_marker_to_tip_m)
        
        # Check Calibration
        if self.debug_skip_calibration:
            if self.calib.load():
                self.desk_plane_depth = self.calib.plane_equation
            self.state = "MAIN"
        else:
            if self.calib.load():
                self.desk_plane_depth = self.calib.plane_equation
                self.state = "MAIN"
                self.status_update.emit("Calibration loaded. Ready.")
            else:
                self.state = "CALIBRATE"
                self.status_update.emit("No calibration found. Please calibrate.")

        if self.desk_plane_depth is not None:
            self.update_world_transform()

    def update_world_transform(self):
        self.R_w, self.origin_w = DeskCalibration.get_world_transform(self.desk_plane_depth)
        self.track_processor.update_transform(self.desk_plane_depth, self.R_w, self.origin_w)

    def initialize_camera(self):
        if self.camera_type == "realsense":
            from ir_pen_tracker.io.realsense_camera import RealSenseCamera
            rs_cfg = self.config.get("camera", {}).get("realsense", {})
            fps = int(rs_cfg.get("fps", 30))
            enable_ir = bool(rs_cfg.get("enable_ir", True))
            preset = str(rs_cfg.get("preset", "high_accuracy")).lower()
            laser_power = rs_cfg.get("laser_power", None)
            exposure = rs_cfg.get("exposure", None)
            
            depth_width = int(rs_cfg.get("depth_width", 1280))
            depth_height = int(rs_cfg.get("depth_height", 720))
            color_width = int(rs_cfg.get("color_width", 1920))
            color_height = int(rs_cfg.get("color_height", 1080))
            
            self.cam = RealSenseCamera(depth_width=depth_width, depth_height=depth_height,
                                           color_width=color_width, color_height=color_height,
                                           fps=fps, enable_ir=enable_ir, preset=preset,
                                           laser_power=laser_power, exposure=exposure)
        else:
            self.cam = KinectCamera(
                use_raw_color=True,
                color_resolution=pyk4a.ColorResolution.RES_1080P,
                camera_fps=pyk4a.FPS.FPS_30,
                depth_mode=pyk4a.DepthMode.NFOV_UNBINNED
            )
        self.cam.open()

    def run(self):
        self.initialize()
        
        while self.running:
            if self.cam is None:
                time.sleep(0.1)
                continue
                
            frame = self.cam.read_frame()
            if frame is None:
                continue
            
            output_frames = {}
            
            if self.state == "CALIBRATE":
                output_frames = self.calib_processor.process(frame)
            elif self.state == "MAIN":
                tracker_result = self.tracker.track(frame)
                
                # Recording
                if self.rec_manager.is_recording:
                    self.rec_manager.record_frame(frame, tracker_result)
                
                # Visualization
                output_frames = self.track_processor.process(frame, tracker_result, self.cam)
            
            if output_frames:
                self.frames_ready.emit(output_frames)
        
        self.cleanup()

    def stop(self):
        self.running = False
        self.wait()

    def cleanup(self):
        self.rec_manager.stop()
        if self.cam:
            self.cam.close()

    # --- Actions ---
    def toggle_recording(self):
        if self.rec_manager.is_recording:
            self.rec_manager.stop()
        else:
            self.rec_manager.start(self.cam, self.desk_plane_depth)

    def start_calibration(self):
        self.state = "CALIBRATE"
        self.status_update.emit("Entered Calibration Mode")

    def confirm_calibration(self):
        # We need to get the plane from the processor or store it in controller when detected
        # The processor updates its internal state or returns detection status.
        # Ideally, processor should emit signal or we query it.
        # But for simplicity, let's access the detected plane from processor if it's stored there.
        
        # Actually, in the previous implementation, process_calibration updated self.desk_plane_color.
        # Let's fix this interaction. The processor detected the board and we need that info.
        
        detected_plane = self.calib_processor.desk_plane_color
        
        if detected_plane is not None:
            calib_data = self.cam.get_calibration_data()
            extrinsics = calib_data.get("extrinsics_color_to_depth")
            
            if extrinsics:
                self.desk_plane_depth = self.calib.transform_plane_color_to_depth(detected_plane, extrinsics)
                self.calib.plane_equation = self.desk_plane_depth
                self.calib.save()
                
                self.update_world_transform()
                
                self.state = "MAIN"
                self.status_update.emit("Calibration Confirmed. Main Mode.")
            else:
                self.status_update.emit("Error: Missing extrinsics.")
        else:
            self.status_update.emit("Cannot confirm: No board detected.")
