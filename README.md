

## Overview

This project is a camera-based vertical angle detection system using OpenCV. It estimates the tilt of lines within a region of interest (ROI) drawn by the user from live video feed.

---

## Files Summary

### `angle_est.py`
Main entry point that contains the `Calibrate` class and UI logic.

#### Main Capabilities:
- Allows users to define a triangle-shaped ROI (or loads it from `points.txt`).
- Applies Canny edge detection and Hough Transform.
- Clusters similar lines and filters based on vertical angle thresholds.
- Draws both accepted and rejected lines in different colors for clarity.
- Visualizes max vertical angle with arrows and angle arc.

### `helpers.py`
Support utilities for image manipulation and angle calculations.

#### Functions:
- `rotate(img, deg)`: Rotates image by a specified angle.
- `create_binary_quad(points, img_size)`: Creates binary mask of a triangle.
- `apply_roi(mask)`: Ensures ROI mask is valid (0 or 255).
- `cluster_lines(lines, rho_bias, angle_bias)`: Clusters similar Hough lines.
- `angle_est(theta, accept_angle)`: Converts theta to vertical angle in degrees; filters by range.

### `points.txt`
Stores the saved triangle ROI. Each line represents a point: X Y format.
```
3 585
469 585
236 -55
```

---

## Parameters (in `Calibrate.__init__()`)

| Parameter           | Meaning                                                       |
|---------------------|---------------------------------------------------------------|
| `saved_path`        | Path for saving/loading corner points                         |
| `ANGLE_TRIANGLE`    | Apex angle (in degrees) used to define triangle ROI           |
| `ACCEPT`            | Max allowable deviation from **vertical** (± degrees around 90°) |
| `CROP_SIZE`         | Vertical crop height from bottom edge of ROI                  |
| `FLIPCODE`          | OpenCV flip code (1=horizontal, 0=vertical, -1=both)          |
| `CANNY_T1`/`T2`     | Lower/upper thresholds for Canny edge detection               |
| `CANNY_APER`        | Aperture size for Canny                                        |
| `BLUR_KSIZE`        | Kernel size for optional blur                                 |
| `ROTATE_CW_DEG`     | Frame rotation clockwise in degrees                           |
| `HOUGH_RHO`         | Distance resolution of Hough grid                             |
| `HOUGH_THETA`       | Angular resolution of Hough grid (in radians)                 |
| `HOUGH_THRESH`      | Min votes for a line to be detected                           |
| `ANGLE_BIAS`        | Angle tolerance in radians for clustering lines               |
| `RHO_BIAS`          | Distance tolerance in pixels for clustering lines             |
| `W`, `H`            | Image resolution width/height                                 |
| `debug`             | If `True`, shows Canny and ROI mask debug images              |

---

## How to Use

1. Run the program:
   ```bash
   python angle_est.py
   ```

2. Choose `y/n` when prompted to use saved ROI from `points.txt`.
3. If `n`, click left and right base points in the `inputted` window.
   - A top point is automatically calculated from the triangle's apex angle.
   - Press `C` to confirm ROI or `R` to reset.
4. After confirmation, angle detection starts automatically.
5. Press `Q` to quit anytime.

---

## Visual Output

- **Blue Lines**: Accepted vertical lines.
- **Gray Lines**: Rejected lines (outside acceptable angle threshold).
- **Red Arrow/Arc**: Shows max vertical tilt detected.
- **Yellow Line**: Baseline reference.

---

## Debug Mode

Enabled by default. It shows:
- Canny edge image (`Canny` window)
- ROI mask image (`Mask ROI` window)

You can turn it off by setting `self.debug = False` in `Calibrate.__init__()`.

---

## Function Input/Output Summary

```python
rotate(img, deg) → np.ndarray (rotated img)
cluster_lines(lines, rho_bias, angle_bias) → np.ndarray (lines)
create_binary_quad(points, img_size) → np.ndarray (mask)
apply_roi(mask) → np.ndarray (roi)
angle_est(theta, accept_angle) → float | False (angle)
```

---

## Dependencies

- Python 3.7+
- OpenCV (`cv2`)
- NumPy

Install via:
```bash
pip install opencv-python numpy
```

---


### Vertical Angle
The system outputs the **direction angle** of each detected line measured from the y‑axis (0° = vertical, 90° = horizontal).  
`ACCEPT` defines how close that direction angle must be to 90 ° (perfect vertical) for the line to be kept.

### ROI Shape (Upward Triangle)
`ANGLE_TRIANGLE` sets the two equal bottom angles.  
The apex is computed as  
```
x_apex = (x_left + x_right)/2
y_apex = y_left - (base_length/2) * tan(ANGLE_TRIANGLE°)
```  
Because OpenCV’s origin is top‑left, subtracting moves the apex upward on screen.

### Purpose of `CROP_SIZE`
After masking with the triangle ROI, the bottom horizontal edge could still be detected as a line.  
Cropping `CROP_SIZE` pixels away removes that edge, reducing false horizontals.

### Units
| Parameter      | Unit |
|----------------|------|
| ANGLE_TRIANGLE | °    |
| ACCEPT         | °    |
| HOUGH_THETA    | rad  |
| ANGLE_BIAS     | rad  |

──────────────────────────────────────────────
