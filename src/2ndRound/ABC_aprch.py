#!/usr/bin/env python3

import serial
import serial.tools.list_ports
from rplidar import RPLidar, RPLidarException
import time
import sys
import argparse
import cv2
import numpy as np
import math

# ===================== CONFIGURATIONS =====================
# General
RESIZE_WIDTH = 640
DIST_THRESHOLD_CM = 30.0 # Decision distance for vision system

# Vision system
KNOWN_WIDTH_CM = 2.08            # Real-world block width in cm
MIN_CONTOUR_AREA = 500           # Minimum contour size
FOCAL_LENGTH_PX = 750.0         # Adjust based on camera calibration

# HSV color ranges for vision system
lower_green = np.array([35, 80, 50])
upper_green = np.array([85, 255, 255])
lower_red1 = np.array([0, 150, 100])
upper_red1 = np.array([10, 255, 255])
lower_red2 = np.array([170, 150, 100])
upper_red2 = np.array([180, 255, 255])

# Hardware connections
LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_BAUD = 460800
ARDUINO_PORT = "/dev/ttyUSB1"
ARDUINO_BAUD = 9600

# Control system
SERVO_CENTER = 110
MAX_ADJ = 30


# ===================== CONTROL CLASSES & UTILS =====================
class KalmanFilter:
      """A Kalman Filter for fusing IMU data (gyro and magnetometer)."""
      def __init__(self, Q_angle, Q_bias, R_measure):
            self.Q_angle = Q_angle
            self.Q_bias = Q_bias
            self.R_measure = R_measure
            self.angle = 0.0
            self.bias = 0.0
            self.P = [[0.0, 0.0], [0.0, 0.0]]
            self.last_time = time.time()

      def filter(self, new_angle, new_rate, dt):
            """Updates the filter with new sensor data."""
            self.angle += dt * (new_rate - self.bias)
            self.P[0][0] += dt * (dt * self.P[1][1] - self.P[0][1] - self.P[1][0] + self.Q_angle)
            self.P[0][1] -= dt * self.P[1][1]
            self.P[1][0] -= dt * self.P[1][1]
            self.P[1][1] += self.Q_bias * dt
             
            S = self.P[0][0] + self.R_measure
            if S == 0:
                  return self.angle
            K = [self.P[0][0] / S, self.P[1][0] / S]
            y = new_angle - self.angle
             
            self.angle += K[0] * y
            self.bias += K[1] * y
             
            P00_temp = self.P[0][0]
            P01_temp = self.P[0][1]
             
            self.P[0][0] -= K[0] * P00_temp
            self.P[0][1] -= K[0] * P01_temp
            self.P[1][0] -= K[1] * P00_temp
            self.P[1][1] -= K[1] * self.P[1][1]

            return self.angle


class PIDController:
      """A basic PID controller for calculating steering adjustments."""
      def __init__(self, kp, ki, kd):
            self.kp = kp
            self.ki = ki
            self.kd = kd
            self.last_error = 0
            self.integral = 0
            self.integral_limit = 10

      def calculate_output(self, error):
            proportional = self.kp * error
            self.integral += error
            self.integral = max(-self.integral_limit, min(self.integral, self.integral_limit))
            integral_term = self.ki * self.integral
            derivative = self.kd * (error - self.last_error)
            self.last_error = error
            return proportional + integral_term + derivative


def parse_sensor_data(data):
      """Parses a string of serial data into a dictionary of sensor values."""
      values = {}
      parts = data.strip().split(',')
      for part in parts:
            try:
                  key, val = part.split(':')
                  values[key] = float(val)
            except ValueError:
                  pass
      return values


def safe_write(serial_conn, cmd):
      """Write safely to serial without crashing."""
      if serial_conn and serial_conn.is_open:
            try:
                  serial_conn.write(cmd.encode("utf-8"))
            except Exception:
                  pass


# ===================== VISION FUNCTIONS =====================
def color_mask(hsv, color_name):
      """Creates a binary mask for a specified color."""
      if color_name == 'green':
            return cv2.inRange(hsv, lower_green, upper_green)
      elif color_name == 'red':
            m1 = cv2.inRange(hsv, lower_red1, upper_red1)
            m2 = cv2.inRange(hsv, lower_red2, upper_red2)
            return cv2.bitwise_or(m1, m2)
      return None


def shortest_side(obj):
      """Calculates the shortest side of a rotated rectangle."""
      w, h = obj['w_rot'], obj['h_rot']
      short, long = (w, h) if w < h else (h, w)
      return short if long/short > 2 else (w+h)/2


def find_objects(frame, color_name):
      """Detects and returns objects of a specific color."""
      hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
      mask = color_mask(hsv, color_name)
      mask = cv2.erode(mask, None, iterations=2)
      mask = cv2.dilate(mask, None, iterations=2)
      contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
      objs = []
      for c in contours:
            if cv2.contourArea(c) < MIN_CONTOUR_AREA:
                  continue
            rect = cv2.minAreaRect(c)
            (rx, ry), (w_rot, h_rot), _ = rect
            box = cv2.boxPoints(rect).astype(int)
            M = cv2.moments(c)
            cx = int(M["m10"]/M["m00"]) if M["m00"] > 0 else int(rx)
            cy = int(M["m01"]/M["m00"]) if M["m00"] > 0 else int(ry)
            objs.append({'cx':cx,'cy':cy,'w_rot':w_rot,'h_rot':h_rot,
                                'box':box,'color':color_name,'contour':c})
      return objs


def process_frame(frame):
      """Processes a video frame to detect colored blocks and measure distance."""
      # Resize frame
      h0, w0 = frame.shape[:2]
      scale = RESIZE_WIDTH / float(w0)
      RESIZE_HEIGHT = int(round(h0 * scale))
      frame = cv2.resize(frame, (RESIZE_WIDTH, RESIZE_HEIGHT))
      fx = FOCAL_LENGTH_PX

      # Detect objects
      red_objs = find_objects(frame, 'red')
      green_objs = find_objects(frame, 'green')
      all_objs = red_objs + green_objs

      nearest = None
      min_dist = float('inf')

      # Analyze objects
      for obj in all_objs:
            width_px = shortest_side(obj)
            if width_px <= 0:
                  continue
            distance_cm = (KNOWN_WIDTH_CM * fx) / width_px

            if distance_cm < min_dist:
                  min_dist = distance_cm
                  nearest = {
                        "color": obj['color'],
                        "dist": distance_cm,
                        "cx": obj['cx'],
                        "cy": obj['cy']
                  }

            # Draw object box
            cv2.drawContours(frame, [obj['box']], 0,
                                      (0,255,0) if obj['color']=='green' else (0,0,255), 2)
            cv2.circle(frame,(obj['cx'],obj['cy']),5,(255,255,255),-1)

      # Decision based on vision
      vision_signal = None
      vision_action_text = "No Obstacle"
      if nearest and nearest["dist"] <= DIST_THRESHOLD_CM:
            if nearest["color"] == "green":
                  vision_action_text = "Move RIGHT (green block)"
                  vision_signal = "A140\n" # Command for a hard right turn
            elif nearest["color"] == "red":
                  vision_action_text = "Move LEFT (red block)"
                  vision_signal = "A80\n" # Command for a hard left turn

      # Add debug text to the frame
      cv2.putText(frame, vision_action_text, (10, RESIZE_HEIGHT - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
      if nearest:
            cv2.putText(frame,
                              f"Nearest: {nearest['color']} {nearest['dist']:.1f}cm",
                              (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

      return vision_signal, frame


# ===================== NAVIGATION FUNCTIONS =====================
def wall_following(scan, pid, servo_center, max_adjustment):
      """Compute steering angle using wall-following."""
      left_distances = [d/1000.0 for q, a, d in scan if 60 <= a <= 90 and d > 100]
      right_distances = [d/1000.0 for q, a, d in scan if 270 <= a <= 300 and d > 100]

      if left_distances and right_distances:
            avg_left = sum(left_distances) / len(left_distances)
            avg_right = sum(right_distances) / len(right_distances)
            error = avg_left - avg_right
            steer = pid.calculate_output(error)
            clamped = max(-1.0, min(steer, 1.0))
            final_angle = servo_center + (clamped * max_adjustment)
            return f"A{int(max(80, min(140, final_angle)))}\n"
      return None


def imu_fallback(imu_data, kalman, dt, servo_center):
      """Compute steering angle using IMU fallback."""
      if "H" not in imu_data or "GY" not in imu_data:
            return None

      imu_data["H"] = 360 - imu_data["H"]
      fused = kalman.filter(imu_data["H"], imu_data["GY"], dt)

      target_heading = 0.0
      error = target_heading - fused
      if error > 180: error -= 360
      elif error < -180: error += 360

      steering_adjustment = error * 0.5
      final_angle = servo_center + steering_adjustment
      return f"A{int(max(80, min(140, final_angle)))}\n"


# ===================== MAIN LOOP =====================
def main():
      print("Starting robot control script...")
      # Controllers
      pid = PIDController(1.5, 0.01, 1.0)
      kalman = KalmanFilter(0.001, 0.003, 0.03)

      # Connect to devices
      print("Initializing hardware...")
      cap = cv2.VideoCapture(("nvarguscamerasrc ! video/x-raw(memory:NVMM), width=640, height=480, format=NV12, framerate=30/1 ! "
                 "nvvidconv flip-method=0 ! video/x-raw, format=BGRx ! "
                 "videoconvert ! video/x-raw, format=BGR ! appsink"), cv2.CAP_GSTREAMER)

      if not cap.isOpened():
            print("? Failed to open camera with GStreamer pipeline")
            sys.exit(1)

      lidar, arduino = None, None
      while lidar is None or arduino is None:
            try:
                  if lidar is None:
                        print("Connecting LiDAR...")
                        lidar = RPLidar(LIDAR_PORT, baudrate=LIDAR_BAUD)
                        lidar.get_info()
                        print("? LiDAR connected")
                  if arduino is None:
                        print("Connecting Arduino...")
                        arduino = serial.Serial(ARDUINO_PORT, ARDUINO_BAUD, timeout=1)
                        time.sleep(2)
                        print("? Arduino connected")
                        safe_write(arduino, "RL\n") # Start motor command
                        print("Sent: RL")
                  break
            except Exception as e:
                  print(f"Connection error: {e}, retrying in 5 seconds...")
                  time.sleep(5)
      
      last_time = time.time()
      is_avoiding = False
      try:
            # Use lidar.iter_scans() as the main loop driver
            for scan in lidar.iter_scans():
                  dt = time.time() - last_time
                  last_time = time.time()

                  # 1. Process vision data
                  ret, frame = cap.read()
                  if not ret:
                        print("? Camera frame not received")
                        continue
                  vision_command, processed_frame = process_frame(frame)

                  # 2. Read IMU data
                  imu_data = {}
                  if arduino.in_waiting:
                        line = arduino.readline().decode("utf-8").strip()
                        imu_data = parse_sensor_data(line)

                  # 3. Make the final decision based on a hierarchy
                  final_command = None
                  
                  # If a block is seen within the threshold, enter avoidance mode
                  if vision_command:
                        is_avoiding = True
                        final_command = vision_command
                  # If we were avoiding, but no block is now visible, switch back to navigation
                  elif is_avoiding:
                        is_avoiding = False
                        final_command = wall_following(scan, pid, SERVO_CENTER, MAX_ADJ) or \
                                                imu_fallback(imu_data, kalman, dt, SERVO_CENTER) or "S\n"
                  # If not in avoidance mode, proceed with normal navigation
                  else:
                        final_command = wall_following(scan, pid, SERVO_CENTER, MAX_ADJ) or \
                                                imu_fallback(imu_data, kalman, dt, SERVO_CENTER) or "S\n"

                  safe_write(arduino, final_command)
                  print(f"Sent: {final_command.strip()}")
                  cv2.imshow("Processed Frame", processed_frame)

                  if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
            
      except KeyboardInterrupt:
            print("Stopping script...")
      finally:
            if lidar:
                  lidar.stop()
                  lidar.stop_motor()
                  lidar.disconnect()
            if arduino and arduino.is_open:
                  safe_write(arduino, "S\n")
                  arduino.close()
            cap.release()
            cv2.destroyAllWindows()
            print("Cleanup done.")

if __name__ == "__main__":
      main()
