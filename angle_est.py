import cv2
import numpy as np
import math
import time
import os
from helpers import rotate, create_binary_quad, apply_roi, cluster_lines, PIDController

class Calibrate:
    def __init__(self):
        # ROI
        self.saved_path = 'points.txt'
        self.ANGLE_TRIANGLE = math.radians(70)
        self.corner_points = []
        self.roi_created = False
        # Img properties
        self.CROP_SIZE = 100
        self.FLIPCODE = 1
        self.ROTATE_CW_DEG = 0
        self.W, self.H = 480, 640
        self.H_CROP = self.H - self.CROP_SIZE
        # Tunning
        self.CANNY_T1 = 40
        self.CANNY_T2 = 120
        self.CANNY_APER = 3
        self.BLUR_KSIZE = 5
        self.ACCEPT = 25
        self.HOUGH_RHO = 1
        self.HOUGH_THETA = np.pi / 180
        self.HOUGH_THRESH = 120
        self.ANGLE_BIAS = 0.3
        self.RHO_BIAS = 20
        # PID
        self.Kp=1.2; self.Ki=0.8; self.Kd=0.05
        self.deadband=2.0
        self.rpm_min=30; self.rpm_max=100; self.base_rpm=65
        

    def on_click_roi(self, event, x, y, *_):
        if event == cv2.EVENT_LBUTTONDOWN and len(self.corner_points) < 2:
            if len(self.corner_points) == 0:
                self.corner_points.append([x, y])
            elif x > self.corner_points[0][0]:
                self.corner_points.append([x, self.corner_points[0][1]])

    def save_points(self):
        np.savetxt(self.saved_path, np.array(self.corner_points), fmt='%d')
        print(f"Saved corner points to {self.saved_path}")

    def load_points(self):
        if os.path.exists(self.saved_path):
            self.corner_points = np.loadtxt(self.saved_path, dtype=int).tolist()
            if isinstance(self.corner_points[0], int):
                self.corner_points = [self.corner_points]
            print(f"Loaded corner points from {self.saved_path}")
            if len(self.corner_points) == 3:
                self.roi_created = True

    def get_roi(self):
        choice = input("Use saved corner points? (y/n): ").strip().lower()
        if choice == 'y':
            self.load_points()
            return

        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            raise IOError("Cannot open camera")

        cv2.namedWindow('inputted')
        cv2.namedWindow('raw')
        cv2.setMouseCallback('inputted', self.on_click_roi)

        while True:
            ok, frame = cap.read()
            if not ok:
                break

            frame = rotate(frame, self.ROTATE_CW_DEG)
            frame = cv2.flip(frame, self.FLIPCODE)
            frame = cv2.resize(frame, (self.W, self.H))

            raw = frame.copy()
            for p in self.corner_points:
                cv2.circle(raw, tuple(p), 5, (0, 0, 255), -1)
                cv2.putText(raw, f"{p[0]},{p[1]}", tuple(p),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)

            if len(self.corner_points) == 2:
                p1, p2 = self.corner_points
                distance = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
                x_3 = int(round((p1[0] + p2[0]) / 2))
                height = (distance / 2) * math.tan(self.ANGLE_TRIANGLE)
                y_3 = int(round(p1[1] - height))
                apex = [x_3, y_3]
                self.corner_points.append(apex)
                cv2.circle(raw, (x_3, p1[1]), 5, (0, 0, 255), -1)
                cv2.circle(raw, (x_3, y_3), 5, (0, 0, 255), -1)

            if len(self.corner_points) == 3:
                cv2.line(raw, self.corner_points[0], self.corner_points[1], (0, 255, 255), 2)
                cv2.line(raw, self.corner_points[0], self.corner_points[2], (0, 255, 255), 2)
                cv2.line(raw, self.corner_points[1], self.corner_points[2], (0, 255, 255), 2)

            cv2.imshow('raw', frame)
            cv2.imshow('inputted', raw)

            k = cv2.waitKey(1) & 0xFF
            if k in (ord('q'), ord('Q')):
                break
            if k in (ord('c'), ord('C')) and len(self.corner_points) == 3:
                self.roi_created = True
                break
            if k in (ord('r'), ord('R')):
                self.corner_points.clear()

        cap.release()
        cv2.destroyAllWindows()

        if self.roi_created:
            save = input("Save corner points to file? (y/n): ").strip().lower()
            if save == 'y':
                self.save_points()

    def preprocess_frame(self, frame, roi_mask):
        frame = rotate(frame, self.ROTATE_CW_DEG)
        frame = cv2.flip(frame, self.FLIPCODE)
        frame = cv2.resize(frame, (self.W, self.H))

        y_bottom = max(self.corner_points[0][1], self.corner_points[1][1])
        y_crop = max(0, y_bottom - self.CROP_SIZE)
        mask_3ch = cv2.merge([roi_mask] * 3)
        frame = np.where(mask_3ch == 255, frame, 255).astype(np.uint8)
        frame = frame[:y_crop, :]
        return frame

    def detect_edges(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, self.CANNY_T1, self.CANNY_T2, apertureSize=self.CANNY_APER)
        return edges

    def detect_lines(self, edges):
        return cv2.HoughLines(edges, self.HOUGH_RHO, self.HOUGH_THETA, self.HOUGH_THRESH)

    def draw_detected_lines(self, frame, flines):
        min_ang = 180
        for rho, theta in flines:
            angle_x_axis = math.degrees(np.pi / 2 - theta)
            if -self.ACCEPT <= angle_x_axis <= self.ACCEPT:
                a, b = np.cos(theta), np.sin(theta)
                x0, y0 = a * rho, b * rho
                pt1 = (int(x0 + 1000 * (-b)), int(y0 + 1000 * a))
                pt2 = (int(x0 - 1000 * (-b)), int(y0 - 1000 * a))
                cv2.line(frame, pt1, pt2, (255, 0, 0), 2)
                min_ang = min(min_ang, angle_x_axis)
        return frame, min_ang

    def draw_overlay(self, frame, angle):
        
        start = (self.W // 2, self.H_CROP - 20)
        end_c = (self.W // 2, int(self.H_CROP - self.H_CROP * 0.4))
        radius = 100
        angle_end = -int(angle)
        dx = int(self.H_CROP * 0.4 * np.cos(math.radians(angle)))
        dy = int(self.H_CROP * 0.4 * np.sin(math.radians(angle)))
        end_h = (start[0] + dx, start[1] - dy)
        cv2.arrowedLine(frame, start, end_h, (0, 0, 255), 2, tipLength=0.15)
        cv2.ellipse(frame, start, (radius, radius), 0, 0, angle_end, (0, 0, 255), 2)
        text_angle = f"{angle:.2f}"
        text_x = int(start[0] + radius * math.cos(math.radians(angle / 2)))
        text_y = int(start[1] - radius * math.sin(math.radians(angle / 2)))
        cv2.putText(frame, text_angle, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)
        cv2.line(frame, start, end_c, (255, 255, 0), 3)
        return frame

    def draw_direction(self, frame, rpm_L, rpm_R):
        # Drawing direction for easier visualize
        diff=abs(rpm_L-rpm_R)
        pt=diff/(self.rpm_max-self.rpm_min)
        leng=200 # 200 px
        color=(255,0,255)
        
        start_y=self.H_CROP-20
        x=20
        if rpm_R>rpm_L:
            left_x_e=(x,int(start_y-leng*(1-pt)))
            right_x_e=(self.W-x,int(start_y-leng*(1+pt)))

        elif rpm_L>rpm_R:
            left_x_e=(x,int(start_y-leng*(1+pt)))
            right_x_e=(self.W-x,int(start_y-leng*(1-pt)))

        else:
            left_x_e=(x,start_y-leng)
            right_x_e=(self.W-x,start_y-leng)

        left_x_s=(x,start_y)
        
        right_x_s=(self.W-x,start_y)

        cv2.line(frame, left_x_s, left_x_e, color, 20)
        cv2.line(frame, right_x_s, right_x_e, color, 20)

    def show_frame(self, window_name, frame):
        cv2.imshow(window_name, frame)

    def main(self):
        mask = create_binary_quad(self.corner_points, img_size=(self.H, self.W))
        roi = apply_roi(mask)
        cap = cv2.VideoCapture('Test.mp4')
        print(f'FPS of the recorded Vid: {cap.get(cv2.CAP_PROP_FPS)}')
        
        #Initialize PID
        pid=PIDController(Kp=self.Kp, Ki=self.Ki, Kd=self.Kd,deadband=self.deadband,rpm_min=self.rpm_min,rpm_max=self.rpm_max,base_rpm=self.base_rpm)
        prev_time=0
        cur_time=0
        while True and self.roi_created:
            ret, frame = cap.read()
            if not ret:
                break

            cur_time=time.time()
            frame = self.preprocess_frame(frame, roi)
            edges = self.detect_edges(frame)
            lines = self.detect_lines(edges)
            hough_vis = frame.copy()

            if lines is not None:
                flines = cluster_lines(lines, self.RHO_BIAS, self.ANGLE_BIAS)
                flines = flines.reshape(len(flines), 2)
                hough_vis, min_ang = self.draw_detected_lines(hough_vis, flines)

                if 90 - min_ang > 0:
                    angle = 90 - min_ang
                    print(f'Vertical Angle: {angle}',end=' ')
                    hough_vis = self.draw_overlay(hough_vis, angle)
                    rpm_L, rpm_R=pid.update(angle)
                    rpm_L, rpm_R=int(rpm_L), int(rpm_R)
                    print(f'Right RPM: {rpm_R:.2f} Left RPM:{rpm_L:.2f}',end=' ')
                    self.draw_direction(hough_vis,rpm_L,rpm_R)

                else:
                    print("Vertical Angle: Not detected!")
            fps=1/(cur_time-prev_time)
            print(f'FPS: {fps}')

            self.show_frame("cam_tester", hough_vis)
            prev_time=cur_time
            if cv2.waitKey(1) & 0xFF in (ord('q'), ord('Q')):
                break

        cap.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    app = Calibrate()
    app.get_roi()
    if app.roi_created:
        app.main()
