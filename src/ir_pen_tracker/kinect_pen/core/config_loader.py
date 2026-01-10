import json
import os
import cv2
import numpy as np

def load_config(config_path="config.json"):
    """
    Loads configuration from a JSON file.
    If the file doesn't exist, returns default configuration.
    """
    defaults = {
        "camera": {
            "type": "kinect",
            "realsense": {
                "preset": "high_accuracy",
                "use_max_resolution": True,
                "fps": 30,
                "enable_ir": True
            }
        },
        "debug": {
            "skip_calibration": False
        },
        "calibration_board": {
            "rows": 8,
            "cols": 12,
            "square_length": 0.024,
            "marker_length": 0.018,
            "dictionary_id": "DICT_4X4_100"
        },
        "pen": {
            "ir_threshold": 60000,
            "min_area": 10,
            "max_area": 2000,
            "roi_scale": 0.5,
            "sample_margin": 0.2,
            "pen_length_m": 0.15,
            "length_tolerance_m": 0.05,
            "ransac_threshold": 0.005,
            "min_depth_m": 0.1,
            "max_depth_m": 3.0
        }
    }

    if not os.path.exists(config_path):
        # Try looking in parent directories or typical locations
        possible_paths = [
            os.path.join("..", config_path),
            os.path.join("..", "..", config_path),
            os.path.join(os.path.dirname(__file__), "..", "..", "..", config_path)
        ]
        for p in possible_paths:
            if os.path.exists(p):
                config_path = p
                break
        else:
            print(f"Config file {config_path} not found. Using defaults.")
            return defaults

    try:
        with open(config_path, 'r') as f:
            user_config = json.load(f)
            
        # Merge user config with defaults (shallow merge for top-level keys)
        # Deep merge would be better but simple is fine for now
        config = defaults.copy()
        for key, value in user_config.items():
            if isinstance(value, dict) and key in config:
                config[key].update(value)
            else:
                config[key] = value
                
        return config
    except Exception as e:
        print(f"Error loading config: {e}. Using defaults.")
        return defaults

def get_aruco_dict(dict_name):
    """
    Resolves ArUco dictionary ID from string name.
    """
    if hasattr(cv2.aruco, dict_name):
        return getattr(cv2.aruco, dict_name)
    # Fallback/Manual mapping if needed
    mapping = {
        "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
        "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
        "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
        "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
        "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
        "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
        "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
        "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
        "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
        "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
        "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
        "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
        "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
        "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
        "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
        "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
        "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
        "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
        "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
        "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
        "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11,
    }
    return mapping.get(dict_name, cv2.aruco.DICT_4X4_100)
