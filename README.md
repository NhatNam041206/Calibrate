# Camera Line-Angle Calibration & PID Control

This toolkit measures the tilt of parallel floor tiles—or any nearly horizontal lines—live on camera, then feeds that angle into a PID controller that balances left‑ and right‑wheel RPMs.

---

## Table of Contents

- [File Overview](#file-overview)
- [Features](#features)
- [Installation](#installation)
- [Getting Started](#getting-started)
- [Key Parameters](#key-parameters-all-in-angle_estpy)
- [Algorithm Notes](#algorithm-notes)
- [Saving & Loading points.txt](#saving--loading-pointstxt)
- [FAQ](#faq)
- [Licence](#licence)
- [Acknowledgements](#acknowledgements)

## File Overview

| File           | Purpose                                                                                                                                                                                                                          |
| -------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `angle_est.py` | **Main application.** Provides the `Calibrate` class that handles ROI definition, image pre‑processing, edge & line detection, clustering, PID control, and rich visualisation of results. Run this file to start the live demo. |
| `helpers.py`   | Helper functions: image rotation, binary ROI mask generation, line clustering, and a robust **PIDController** tuned for ±90 ° alignment tasks.                                                                                   |
| `points.txt`   | Stores the three ROI corner points (two base points + apex) so you don’t have to click them every session. The file is auto‑generated the first time you define an ROI.                                                          |

---

## Features

1. **Interactive ROI selection**
   - Click the two base corners of the inspection triangle; the apex is auto‑computed using `ANGLE_TRIANGLE` (default 70 °).
   - Press **C** to confirm, **R** to reset, **Q** to quit.
   - Optionally save to `points.txt` for future runs.
2. **Robust line extraction**
   - Gaussian blur → Canny edge detector → Standard Hough transform.
   - Automatic line clustering (`cluster_lines`) removes near‑duplicates by ρ & θ.
3. **Precise vertical‑angle computation**
   - Converts every detected line’s θ to its angle with respect to the image x‑axis.
   - Ignores lines whose angle deviates more than `ACCEPT` (25 ° by default) from horizontal.
4. **PID‑based wheel control**
   - Error = desired vertical (90 °) − detected angle.
   - Includes dead‑band, anti‑wind‑up, and RPM limiting.
5. **Live visualisation**
   - Displays detected lines, angle arc, numeric read‑out, and animated wheel‑RPM bars in real time.

---

## Installation

```bash
python -m venv venv
source venv/bin/activate        # Windows: venv\\Scripts\\activate
pip install -r requirements.txt # Requires opencv‑contrib‑python, numpy
```

> **Python 3.9+** is recommended because `time.perf_counter()` behaves consistently and NumPy typing is supported.

---

## Getting Started

```bash
python angle_est.py
```

1. When prompted to use saved points, choose **n** on the first run and click the two base corners (left → right). Press **C** when you see the cyan triangle.
2. The live camera feed (device 0 by default) now shows:
   - Cyan ROI mask (cropped at `CROP_SIZE` from the bottom).
   - Red arrow = detected tilt.
   - Blue horizontal baseline and red arc = angle magnitude.
   - Magenta vertical bars at the edges = mapped wheel RPMs.
3. Press **Q** to exit.

---

## Key Parameters (all in `angle_est.py`)

| Name                             | Meaning                            | Typical Range          |
| -------------------------------- | ---------------------------------- | ---------------------- |
| `ANGLE_TRIANGLE`                 | Half‑apex of triangular ROI        | 60 ° – 75 °            |
| `CROP_SIZE`                      | Vertical pixels excluded below ROI | 50 – 150               |
| `CANNY_T1`, `CANNY_T2`           | Canny thresholds                   | 10‑80 / 100‑200        |
| `HOUGH_THRESH`                   | Votes for a valid Hough line       | 80‑150                 |
| `RHO_BIAS`, `ANGLE_BIAS`         | Line‑cluster tolerances            | 10‑30 px / 0.2‑0.4 rad |
| `Kp`, `Ki`, `Kd`                 | PID gains                          | tune for your robot    |
| `rpm_min`, `rpm_max`, `base_rpm` | Wheel RPM limits                   | depends on motors      |

All parameters are grouped at the top of each file for quick editing.

---

## Algorithm Notes

### 1. Triangle ROI

- The user clicks base points **P1** and **P2**. The apex **P3** is computed so the triangle opens upward by `ANGLE_TRIANGLE`.
- This keeps the ROI inside the image even if the camera is tilted; the apex is always above the base midpoint.

### 2. Line Clustering

`cluster_lines` merges Hough lines whose (ρ, θ) values are within the specified tolerances. The mean of each cluster is used so duplicate detections do not skew the PID input.

### 3. Angle Definition

- **Vertical angle** = 90 ° – θₓ, where θₓ is the line angle with respect to the x‑axis.
- Positive values → right tilt; negative values → left tilt.
- The `ACCEPT` window filters out noisy near‑horizontal edges that do not correspond to floor tiles.

### 4. PID Mapping

```text
err       = 90° – angle          # degrees
u         = Kp·P + Ki·I + Kd·D   # PID output
rpm_left  = base_rpm – u
rpm_right = base_rpm + u
```

A dead‑band of ±2 ° prevents twitching when the robot is already aligned.

---

## Saving & Loading points.txt

```
<x1> <y1>
<x2> <y2>
<x3> <y3>
```

The file is read automatically when you answer **y** to “Use saved corner points?”.

---

## FAQ

- **Why does the arrow sometimes disappear?**\
  No valid horizontal line was found within the `ACCEPT` band. Increase `ACCEPT` or improve lighting.

- **Vertical angle sign seems inverted!**\
  Swap the corner‑click order (left first) or change `FLIPCODE`.

- **PID is too aggressive or too slow.**\
  Adjust `Kp`, reduce `Ki`, or increase `Kd`.

---

