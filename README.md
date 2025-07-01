# Camera Line-Angle Calibration & PID Control

This toolkit measures the tilt of parallel floor tiles—or any nearly horizontal lines—live on camera, then feeds that angle into a PID controller that balances left‑ and right‑wheel RPMs. Originally built for a small mobile robot, its modules are generic enough to drop into any vision‑based alignment project.

---

## 📑 Table of Contents

- [File Overview](#file-overview)
- [Features](#features)
- [Installation](#installation)
- [Getting Started](#getting-started)
- [Key Parameters](#key-parameters-all-in-angle_estpy)
- [Algorithm Notes](#algorithm-notes)
- [Saving & Loading ](#saving--loading-pointstxt)[`points.txt`](#saving--loading-pointstxt)
- [FAQ](#faq)
- [Licence](#licence)
- [Acknowledgements](#acknowledgements)

## 📂 File Overview

| File           | Purpose                                                                                                                                                                                                                          |
| -------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `angle_est.py` | **Main application.** Provides the `Calibrate` class that handles ROI definition, image pre-processing, edge & line detection, clustering, PID control, and rich visualisation of results. Run this file to start the live demo. |
| `helpers.py`   | Collection of reusable helpers: rotation wrapper, binary ROI creation, line clustering, and a robust **PIDController** tuned for ±90 ° alignment tasks.                                                                          |
| `points.txt`   | Stores the three ROI corner points (two base points + apex) in plain text so you don’t need to click them every session. Will be auto-generated the first time you define an ROI.                                                |

---

## ✨ Features

1. **Interactive ROI selection**
   - Click the two base corners of the inspection triangle; the apex is auto-computed using `ANGLE_TRIANGLE` (default 70°).
   - Press **C** to confirm, **R** to reset, **Q** to quit.
   - Optionally save to `points.txt` for future runs.
2. **Robust line extraction**
   - Gaussian blur → Canny edge detector → Standard Hough transform.
   - Automatic line clustering (`cluster_lines`) removes near-duplicates by ρ & θ.
3. **Precise vertical-angle computation**
   - Converts every detected line’s θ to its angle w\.r.t. the image x-axis.
   - Ignores lines whose angle deviates > `ACCEPT` (25 ° by default) from horizontal.
4. **PID-based wheel control**
   - Error = desired vertical (90 °) − detected angle.
   - Dead-band, anti-wind-up, and RPM limiting built-in.
5. **Live visualisation**
   - Shows detected lines, angle arc, value read-out, and animated wheel-RPM bars.

---

## 🛠️ Installation

```bash
python -m venv venv
source venv/bin/activate  # Windows: venv\\Scripts\\activate
pip install -r requirements.txt  # Requires opencv-contrib-python, numpy
```

> **Python 3.9+** is recommended because `time.perf_counter()` behaves consistently and NumPy typing is supported.

---

## 🚀 Getting Started

```bash
python angle_est.py
```

1. **ROI set-up** – the app first asks whether to use saved points. Choose **n** on the first run and click the two base corners (left → right). Press **C** when a cyan triangle appears.
2. The live camera feed (default device 0) now shows:
   - Cyan ROI mask (cropped at `CROP_SIZE` from bottom).
   - Red arrow = detected tilt.
   - Blue horizontal baseline and red arc = angle magnitude.
   - Magenta vertical bars at screen edges = mapped wheel RPMs.
3. Press **Q** to exit.

---

## ⚙️ Key Parameters (all in `angle_est.py`)

| Name                             | Meaning                            | Typical Range          |
| -------------------------------- | ---------------------------------- | ---------------------- |
| `ANGLE_TRIANGLE`                 | Half-apex of triangular ROI        | 60 ° – 75 °            |
| `CROP_SIZE`                      | Vertical pixels excluded below ROI | 50 – 150               |
| `CANNY_T1`, `CANNY_T2`           | Canny thresholds                   | 10-80 / 100-200        |
| `HOUGH_THRESH`                   | Votes for valid Hough line         | 80-150                 |
| `RHO_BIAS`, `ANGLE_BIAS`         | Line-cluster tolerances            | 10-30 px / 0.2-0.4 rad |
| `Kp`, `Ki`, `Kd`                 | PID gains                          | tune per robot         |
| `rpm_min`, `rpm_max`, `base_rpm` | Wheel RPM limits                   | depends on motors      |

All values are grouped at the top of each file for easy tweaking.

---

## 🧩 Algorithm Notes

### 1. Triangle-ROI

- Base points **P1** and **P2** are user-clicked. The apex **P3** is computed so that the triangle’s base forms the ROI’s lower edge and the sides open upward by `ANGLE_TRIANGLE`.
- This keeps the ROI inside the image even if the camera is tilted because the apex is always above the base midpoint.

### 2. Line Clustering

`cluster_lines` merges Hough lines whose (ρ, θ) differences are within user-defined biases. The mean of each cluster is taken so duplicated detections don’t skew the PID input.

### 3. Angle Definition

- **Vertical Angle** = 90 ° – *θₓ*, where *θₓ* is the line angle w\.r.t. the x-axis.
- Positive values → right-hand tilt; negative → left.
- The `ACCEPT` window filters out noisy near-horizontal edges that don’t correspond to floor tiles.

### 4. PID Mapping

```
err      = 90° − angle  # degrees
u        = Kp·P + Ki·I + Kd·D        # PID output
rpm_Left = base_rpm − u
rpm_Right= base_rpm + u
```

A dead-band of ±2 ° prevents twitching when the robot is already aligned.

---

## 📝 Saving & Loading `points.txt`

```
<x1> <y1>
<x2> <y2>
<x3> <y3>
```

The file uses plain integers and is read automatically when you answer **y** to *“Use saved corner points?”*.

---

## 🤔 FAQ

- **Why does the arrow sometimes disappear?**
  - No valid horizontal line was found within the `ACCEPT` band. Increase `ACCEPT` or improve lighting.
- **Vertical angle sign seems inverted!**
  - Swap the corner-click order (left first) or change `FLIPCODE`.
- **PID is too aggressive/slow.**
  - Tune `Kp`, reduce `Ki`, or increase `Kd`.

---

## 📑 Licence

This code is released under the MIT Licence – do whatever you like, but credit the authors.

---

## 🙏 Acknowledgements

- OpenCV for the robust computer-vision backbone
- NumPy for vectorised line clustering
- The robotics community for endless inspiration 🚗💨

