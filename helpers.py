import numpy as np
import cv2
import math
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

def angle_est(theta,accept_angle=30):
    angle_x_axis = math.degrees(np.pi / 2 - theta)
    if not (- accept_angle <= angle_x_axis <= accept_angle):
        return False

    if 90 - angle_x_axis > 0:
        angle = 90 - angle_x_axis
    else: return False

    return angle