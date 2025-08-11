import numpy as np
import cv2

def find_y(points, cond_val=255):
    mask = (points == cond_val)
    row_has = mask.any(axis=1)
    ys = np.where(row_has)[0]
    if ys.size == 0:
        return -1,-1
    return ys.max(),ys.min()

def find_x(points, start_row, cond_val=255):
    sub = points[:start_row+1]
    col_has = (sub == cond_val).any(axis=0)
    xs = np.where(col_has)[0]
    if xs.size == 0:
        return 0, 0
    return xs.max(), xs.min()

def calculate_area(y_max, x_max, x_min):
    return (y_max + 1) * (x_max - x_min)

def boundary_ob(points, cond_val=255):
    y_max,y_min = find_y(points, cond_val)
    if y_max < 0:
        return None
    x_max, x_min = find_x(points, y_max, cond_val)
    return y_max, y_min,x_max, x_min

# --- Main loop ---
cam = cv2.VideoCapture(1)
o_w, o_h = 640, 480
reduce = 0.5
cam.set(cv2.CAP_PROP_FRAME_WIDTH, int(o_w*reduce)) # Reduce by 0.5
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, int(o_h*reduce))

while True:
    ret, frame = cam.read()
    if not ret:
        break

    blur = cv2.blur(frame, (7, 7))
    gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 50, 130)

    pts = boundary_ob(edges)

    vis = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

    if pts is not None:
        y_max, y_min, x_max, x_min = pts
        cv2.circle(vis, (x_max, y_max), 5, (0, 255, 0), -1)
        cv2.circle(vis, (x_min, y_max), 5, (0, 255, 0), -1)
        cv2.circle(vis, (x_min, y_min), 5, (0, 255, 0), -1)
        cv2.circle(vis, (x_max, y_min), 5, (0, 255, 0), -1)
        cv2.line(vis, (x_min, y_max), (x_max, y_max), (0, 255, 0), 2)
        cv2.line(vis, (x_min, y_max), (x_min, y_min), (0, 255, 0), 2)
        cv2.line(vis, (x_max, y_max), (x_max, y_min), (0, 255, 0), 2)
        cv2.line(vis, (x_min, y_min), (x_max, y_min), (0, 255, 0), 2)
        print(f'Area: {calculate_area(y_max, x_max, x_min)}; Percentage of area: {calculate_area(y_max, x_max, x_min) / ((int(o_w*reduce) * int(o_h*reduce))) * 100:.2f}%')
    cv2.imshow('Camera Feed', vis)
    cv2.imshow('Edge', edges)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cam.release()
cv2.destroyAllWindows()
