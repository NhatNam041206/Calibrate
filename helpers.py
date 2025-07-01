import numpy as np
import cv2
import math,time

class PIDController:
    def __init__(self,
                 Kp=1.2, Ki=0.8, Kd=0.05,
                 deadband=2.0,
                 rpm_min=30, rpm_max=100, base_rpm=65):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.dead = deadband
        self.base = base_rpm
        self.rmin, self.rmax = rpm_min, rpm_max
        self.umax = min(base_rpm - rpm_min, rpm_max - base_rpm)

        # trạng thái nội bộ
        self.I  = 0.0
        self.e_prev = 0.0
        self.t_prev = time.perf_counter()

    def update(self, alpha_deg: float):
        # --- dt ---
        now = time.perf_counter()
        dt  = now - self.t_prev or 1e-3
        self.t_prev = now

        # --- dead-band ---
        err = 90.0 - alpha_deg
        if abs(err) <= self.dead:
            e_eff=0.0
        else:
            e_eff = math.copysign(abs(err) - self.dead, err)

        # --- PID ---
        P = self.Kp * e_eff
        if abs(e_eff) > 1e-3:                # tránh wind-up trong dead-band
            self.I += self.Ki * e_eff * dt
        D = self.Kd * (e_eff - self.e_prev) / dt
        self.e_prev = e_eff
        u = max(-self.umax, min(self.umax, P + self.I + D))

        # --- mapping ---
        rpm_L = max(self.rmin, min(self.rmax, self.base - u))
        rpm_R = max(self.rmin, min(self.rmax, self.base + u))
        return rpm_L, rpm_R
    
def rotate(img,ROTATE_CW_DEG):
    d = ROTATE_CW_DEG % 360
    if d == 90:  return cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
    if d == 180: return cv2.rotate(img, cv2.ROTATE_180)
    if d == 270: return cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
    return img

def cluster_lines(lines,RHO_BIAS,ANGLE_BIAS):
    visited=[]
    lines_filtered=[]
    for i in range(len(lines)):
        if list(lines[i][0]) in visited:
            continue

        cluster, clustered = [], False
        for j in range(i, len(lines)):
            angleD = abs(lines[i][0][1] - lines[j][0][1])
            pD     = abs(lines[i][0][0] - lines[j][0][0])

            if angleD < ANGLE_BIAS and pD < RHO_BIAS:
                cluster.extend([lines[j]])
                visited.append(list(lines[j][0]))
                clustered = True

        # keep average   or   raw line
        out = np.mean(cluster, axis=0) if clustered else lines[i]
        lines_filtered.append(out)
    return np.array(lines_filtered)

def create_binary_quad(points, img_size=(480,640)):

    mask = np.zeros(img_size, dtype=np.uint8)

    pts = np.array(points, dtype=np.int32).reshape(-1, 1, 2)

    cv2.fillPoly(mask, [pts], 255)

    return mask

def apply_roi(mask):
    # Ensure mask is binary 0 or 255
    mask = np.clip(mask, 0, 255).astype(np.uint8)
    return mask

def angle_est(theta, accept_angle=30, debug=False):
    deg = math.degrees(theta)
    if deg>90+accept_angle or deg<90-accept_angle: return False
    return deg

