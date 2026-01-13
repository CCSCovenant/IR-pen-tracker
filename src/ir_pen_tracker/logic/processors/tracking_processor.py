import cv2
import numpy as np
from ir_pen_tracker.core.utils import CVUtils
from typing import Any, cast
import pyrealsense2 as rs
rs = cast(Any, rs)


class TrackingProcessor:
    def __init__(self, config, ortho_vis, lowest_marker_to_tip_m):
        self.config = config
        self.ortho_vis = ortho_vis
        self.lowest_marker_to_tip_m = lowest_marker_to_tip_m
        self.desk_plane_depth = None
        self.R_w = None
        self.origin_w = None

    def update_transform(self, desk_plane_depth, R_w, origin_w):
        self.desk_plane_depth = desk_plane_depth
        self.R_w = R_w
        self.origin_w = origin_w

    def draw_points_on_view(self, view_bgr, rs_intrinsics, tip_m, tail_m, real_tip_m, corrected_tip_m, corr_color_bgr):
        u_tip, v_tip = rs.rs2_project_point_to_pixel(rs_intrinsics, [float(tip_m[0]), float(tip_m[1]), float(tip_m[2])])
        u_tail, v_tail = rs.rs2_project_point_to_pixel(rs_intrinsics, [float(tail_m[0]), float(tail_m[1]), float(tail_m[2])])
        u_real, v_real = rs.rs2_project_point_to_pixel(rs_intrinsics, [float(real_tip_m[0]), float(real_tip_m[1]), float(real_tip_m[2])])
        u_corr, v_corr = rs.rs2_project_point_to_pixel(rs_intrinsics, [float(corrected_tip_m[0]), float(corrected_tip_m[1]), float(corrected_tip_m[2])])

        tip_uv = (int(u_tip), int(v_tip))
        tail_uv = (int(u_tail), int(v_tail))
        real_uv = (int(u_real), int(v_real))
        corr_uv = (int(u_corr), int(v_corr))

        cv2.line(view_bgr, tip_uv, tail_uv, (0, 255, 255), 2)
        cv2.circle(view_bgr, tip_uv, 4, (0, 0, 255), -1)
        cv2.circle(view_bgr, tail_uv, 4, (255, 0, 0), -1)
        cv2.line(view_bgr, tip_uv, real_uv, (200, 200, 200), 2)
        cv2.circle(view_bgr, real_uv, 4, (255, 255, 255), -1)
        cv2.circle(view_bgr, corr_uv, 5, corr_color_bgr, -1)

    def process(self, frame, tracker_result, cam_instance):
        cam_view = frame.color.copy()
        ir_view = cv2.cvtColor(CVUtils.ir_to_vis(frame.ir), cv2.COLOR_GRAY2BGR)

        extr_d2c = cam_instance.get_calibration_data()["extrinsics_depth_to_color"]
        rs_color_intr = cam_instance.get_rs_color_intrinsics()
        rs_ir_intr = cam_instance.get_rs_ir_left_intrinsics()

        if tracker_result.has_lock:
            tip_d = tracker_result.tip_pos_cam
            direction_d = tracker_result.direction
            tail_d = tracker_result.tail_pos_cam
            real_tip_d = tip_d - direction_d * float(self.lowest_marker_to_tip_m)
            
            n = self.desk_plane_depth[:3]
            d_param = self.desk_plane_depth[3]
            dist_real = np.dot(n, real_tip_d) + d_param
            is_touching = dist_real > 0
            corrected_tip_d = real_tip_d
            v = real_tip_d - tip_d
            denom = np.dot(n, v)
            dist_tip = np.dot(n, tip_d) + d_param
            t = -dist_tip / denom
            if is_touching:
                corrected_tip_d = tip_d + t * v
            compression_mm = np.linalg.norm(corrected_tip_d - real_tip_d) * 1000.0

            p_w = self.R_w @ (corrected_tip_d - self.origin_w)
            self.ortho_vis.update(p_w[0], p_w[1], p_w[2], hover_height=p_w[1])

            tip_c = CVUtils.transform_point(tip_d, extr_d2c)
            tail_c = CVUtils.transform_point(tail_d, extr_d2c)
            real_tip_c = CVUtils.transform_point(real_tip_d, extr_d2c)
            corrected_tip_c = CVUtils.transform_point(corrected_tip_d, extr_d2c)

            corr_color = (255, 0, 255) if is_touching else (0, 255, 255)
            self.draw_points_on_view(cam_view, rs_color_intr, tip_c, tail_c, real_tip_c, corrected_tip_c, corr_color)
            self.draw_points_on_view(ir_view, rs_ir_intr, tip_d, tail_d, real_tip_d, corrected_tip_d, corr_color)
            cv2.putText(cam_view, f"{compression_mm:.1f}mm", (50, 140), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 255), 2)

        if tracker_result.has_lock:
            cv2.putText(cam_view, "TRACKING", (50, 80), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 3)
        else:
            cv2.putText(cam_view, "LOST", (50, 80), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 3)

        ortho_img = self.ortho_vis.draw()
        
        return {
            "color": cam_view,
            "ir": ir_view,
            "ortho": ortho_img
        }
