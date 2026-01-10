# IR Tracking Specification for Strip-type Pen

## Overview
This document outlines the technical specification for implementing an Infrared (IR) based tracking algorithm for a strip-type pen input device using Azure Kinect.
The target object is a pen with two IR-reflective strips:
1. **Tip**: Located at the writing end.
2. **Tail**: Located at the upper end.

The goal is to estimate the 6DoF pose (position and orientation) of the pen.

## Input Data
- **IR Image**: 16-bit grayscale image from Azure Kinect IR camera.
- **Depth Image**: 16-bit depth map (millimeters) aligned to the IR camera.
- **Camera Intrinsics**: Focal length ($f_x, f_y$) and principal point ($c_x, c_y$).

## Algorithm Pipeline

### 1. Preprocessing & Segmentation
- **Thresholding**: Apply a high intensity threshold to the IR image to isolate reflective markers.
  - `ir_threshold`: Configurable parameter (default ~3000-4000 depending on environment/material).
- **Morphological Operations**: Optional opening/closing to remove salt-and-pepper noise and fill gaps in strips.
- **Connected Components (Blob Detection)**:
  - Find contours or connected components in the binary mask.
  - Filter blobs based on:
    - **Area**: Min/Max pixel area to reject small noise or large non-pen reflections.
    - **Aspect Ratio**: Since strips might appear elongated, aspect ratio filtering might be useful (optional).

### 2. 3D Reconstruction
For each valid blob:
1. **Centroid**: Calculate the weighted centroid $(u, v)$ in image coordinates.
2. **Depth Estimation**:
   - Sample depth values in a window around $(u, v)$ from the Depth Image.
   - Compute **Median Depth** ($z$) to reject outliers (flying pixels).
   - Reject blobs with invalid or zero depth.
3. **Back-projection**:
   - Convert $(u, v, z)$ to 3D point $P = (x, y, z)$ using camera intrinsics:
     $$x = \frac{(u - c_x) \cdot z}{f_x}, \quad y = \frac{(v - c_y) \cdot z}{f_y}$$

### 3. Pen Identification & Pose Estimation
The system assumes we are looking for exactly two markers (Tip and Tail) with a known physical distance.

1. **Pairwise Matching**:
   - If $>2$ blobs are detected, generate all pairs.
   - Calculate Euclidean distance $d_{ij} = \|P_i - P_j\|$.
   - Select the pair $(P_i, P_j)$ where $|d_{ij} - L_{pen}| < \epsilon$.
     - $L_{pen}$: Physical distance between Tip and Tail centers.
     - $\epsilon$: Tolerance (e.g., 2cm).

2. **Tip/Tail Disambiguation**:
   - **Intensity/Size**: If one strip is significantly larger/brighter, use that property.
   - **Tracking**: Use Kalman Filter or simple velocity tracking to maintain identity across frames.
   - **Heuristic**: If tracking is lost, assume the "Tip" is the point closer to the camera or further (depending on usage), or rely on previous known state. 
   - *Proposed Approach*: 
     - Initialize: Assume finding a valid pair is enough.
     - For the very first frame or after loss, we might have ambiguity.
     - Ambiguity Resolution: The user might need to point the pen in a specific way (e.g. Tip up) or we assume the Tip is the endpoint that moves less (if writing) or similar. 
     - **Simpler approach for v1**: Assume the pen length constraint is unique enough. Use the previous frame's Tip position to identify the new Tip (nearest neighbor).

### 4. Output
- **Tip Position**: $P_{tip}$
- **Direction**: Normalized vector $\vec{v} = \frac{P_{tail} - P_{tip}}{\|P_{tail} - P_{tip}\|}$
- **Timestamp**: Frame timestamp.

## Configuration Parameters (`IRPenConfig`)
| Parameter | Type | Description | Default |
|-----------|------|-------------|---------|
| `ir_threshold` | int | Min IR brightness value | 3000 |
| `min_area` | int | Min blob area (pixels) | 10 |
| `max_area` | int | Max blob area (pixels) | 1000 |
| `pen_length` | float | Distance between Tip and Tail (meters) | 0.15 (Example) |
| `length_tolerance`| float | Allowed deviation in length (meters) | 0.03 |
| `tracking_window` | int | Depth window size | 5 |

## Implementation Details
- **Class**: `IRPenTracker` implementing `IBrushTracker`.
- **File**: `src/kinect_pen/algo/ir_pen_tracker.py` (or added to `pen_tracker.py`).
- **Dependencies**: `numpy`, `cv2`, `dataclasses`.

