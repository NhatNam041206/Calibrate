README.txt
──────────────────────────────────────────────
Project: Real-Time Vertical Angle Estimator
Author: [Your Name]
Last Updated: [Today’s Date]
──────────────────────────────────────────────

Overview
--------
This project provides a robust pipeline for real-time estimation of vertical angles from live video feed using OpenCV. It enables users to manually define a Region of Interest (ROI) and automatically detects line segments within that region to estimate the tilt angle from the vertical axis.

It is especially useful for robotics, visual guidance systems, or any application requiring precise angle alignment from a camera feed.

Files
-----

1. angle_est.py
   ─────────────
   Main application file containing the `Calibrate` class.

   Responsibilities:
   - ROI setup (manual or from file)
   - Edge and line detection using Canny + Hough Transform
   - Angle filtering using θ clustering
   - Visual overlay of computed vertical angle
   - Optional save/load for ROI configuration

   Usage:
   ```bash
   python angle_est.py
   ```

2. helpers.py
   ───────────
   Utility functions abstracted to keep the main logic clean.

   Includes:
   - `rotate`: Handle 90°-based image rotations.
   - `create_binary_quad`: Generate a binary mask polygon from 3-point ROI.
   - `apply_roi`: Clean mask before processing.
   - `cluster_lines`: Group similar Hough lines by ρ and θ biases.
   - `angle_est`: Convert θ to angle vs. vertical and apply acceptance criteria.

3. points.txt
   ───────────
   File for persistent storage of corner points.

   Format:
   Each line contains x and y coordinates of the corner points in pixel units.
   ```
   3   585
   469 585
   236 -55
   ```

   Notes:
   - Three points are needed to form a triangle: base-left, base-right, and apex.
   - Created automatically when the user opts to save ROI.

How to Use
----------

1. Run `angle_est.py`. On first run:
   - Choose `n` to manually input ROI via clicking the camera feed.
   - Click base-left, base-right → triangle apex is calculated automatically.
   - Press `C` to confirm, or `R` to reset points.
   - Optionally save to `points.txt`.

2. On future runs:
   - Choose `y` to auto-load previously saved ROI from `points.txt`.

3. During line detection:
   - Uses `cv2.HoughLines` to detect lines within the ROI.
   - Filters lines via `cluster_lines()` based on similarity thresholds.
   - Uses `angle_est()` to estimate deviation from vertical.

4. Visualization:
   - Live display shows lines and the angle visualization arc.
   - Arrow and ellipse illustrate the vertical angle clearly.

Tuning Parameters
-----------------

Defined in `Calibrate.__init__()`:
- `CROP_SIZE`: Remove lower region noise.
- `CANNY_T1/T2/APER`: Edge detection thresholds.
- `HOUGH_THRESH`: Minimum Hough votes to consider a line.
- `ANGLE_BIAS`, `RHO_BIAS`: Clustering thresholds for line filtering.
- `ACCEPT`: Acceptable range for near-vertical angles (± degrees).
- `ROTATE_CW_DEG`: Set camera frame orientation (90, 180, 270).

Dependencies
------------
- Python 3.7+
- OpenCV (cv2)
- NumPy

Installation
------------
Install dependencies via pip:
```bash
pip install numpy opencv-python
```

Known Issues
------------
- Only tested on camera input with 640x480 resolution.
- ROI input is mouse-based and not currently persistent between sessions unless saved.

License
-------
MIT License (or add your preferred license)

──────────────────────────────────────────────
Professional Tip:
This tool is modular enough to embed in embedded vision pipelines (e.g., Jetson Nano or Raspberry Pi) with minor tweaks to frame capture logic.
──────────────────────────────────────────────
