#!/usr/bin/env python3

import serial
import serial.tools.list_ports
from rplidar import RPLidar, RPLidarException
import time
import sys
import argparse
import re
import math

# --- PID Controller Class ---
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
    
    def reset(self):
        """Resets the integral and last_error terms."""
        self.integral = 0
        self.last_error = 0

# --- Utility functions ---
def parse_sensor_data(data):
    """
    Parses a string of serial data into a dictionary of sensor values.
    Example input: "H:123.45"
    """
    values = {}
    parts = data.strip().split(':')
    if len(parts) == 2 and parts[0] == 'H':
        try:
            values['H'] = float(parts[1])
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

# --- Control logic ---
def wall_following(scan, pid_lidar, pid_imu, servo_center, max_adjustment, imu_data):
    """Compute steering angle using a combination of LiDAR and IMU."""
    left_distances = [d/1000.0 for q, a, d in scan if 60 <= a <= 90 and d > 100]
    right_distances = [d/1000.0 for q, a, d in scan if 270 <= a <= 300 and d > 100]

    # Prioritize wall-following with LiDAR
    if left_distances and right_distances:
        # Reset IMU PID state when we can rely on LiDAR again
        pid_imu.reset()
        
        avg_left = sum(left_distances) / len(left_distances)
        avg_right = sum(right_distances) / len(right_distances)
        error = avg_left - avg_right
        steer = pid_lidar.calculate_output(error)
        clamped = max(-1.0, min(steer, 1.0))
        final_angle = servo_center + (clamped * max_adjustment)
        return f"A{int(max(80, min(140, final_angle)))}\n"
    
    # If LiDAR data is sparse, fall back to using the IMU heading for basic steering
    elif "H" in imu_data:
        # Reset LiDAR PID state when we switch to IMU control
        pid_lidar.reset()
        
        current_heading = imu_data["H"]
        target_heading = 0.0 # Or some other target for straight driving
        error = target_heading - current_heading

        if error > 180: error -= 360
        elif error < -180: error += 360

        # Use the IMU PID controller for steering
        steering_adjustment = pid_imu.calculate_output(error)
        final_angle = servo_center + steering_adjustment
        return f"A{int(max(80, min(140, final_angle)))}\n"
        
    return None

# --- Main Loop ---
def main():
    print("Starting robot control script...")

    # Controllers
    pid_lidar = PIDController(1.75, 0.001, 1.0) # Tuned for LiDAR wall following
    pid_imu = PIDController(0.5, 0.0, 0.2) # New PID for IMU-based steering

    # Config
    SERVO_CENTER = 110
    MAX_ADJ = 30
    LIDAR_PORT = "/dev/ttyUSB0"
    LIDAR_BAUD = 460800
    ARDUINO_PORT = "/dev/ttyUSB1" # Common port for Arduino, or use "ls /dev/tty*" to find it
    ARDUINO_BAUD = 9600

    # Connect devices
    lidar, arduino = None, None
    while True:
        try:
            if lidar is None:
                print("Connecting LiDAR...")
                lidar = RPLidar(LIDAR_PORT, baudrate=LIDAR_BAUD)
                lidar.get_info()
                print("LiDAR connected")
            if arduino is None:
                print("Connecting Arduino...")
                arduino = serial.Serial(ARDUINO_PORT, ARDUINO_BAUD, timeout=1)
                time.sleep(2)
                print("Arduino connected")
                # Send the "ready" command to light up the LED
                command = "RL\n"
                safe_write(arduino, command)
                print("Sent:", command.strip())
            break
        except Exception as e:
            print(f"Connection error: {e}, retrying...")
            if lidar:
                lidar.stop_motor()
                lidar.disconnect()
                lidar = None
            if arduino:
                arduino.close()
            arduino = None
            time.sleep(5)

    try:
        last_heartbeat_time = time.time()
        for scan in lidar.iter_scans():
            # Send a periodic "heartbeat" to the Arduino to show the device is active
            if time.time() - last_heartbeat_time > 5: # Send heartbeat every 5 seconds
                safe_write(arduino, "RL\n")
                last_heartbeat_time = time.time()
            
            # Read IMU data from Arduino
            imu_data = {}
            if arduino.in_waiting:
                line = arduino.readline().decode("utf-8").strip()
                imu_data = parse_sensor_data(line)
                
            # Compute steering angle based on available sensor data
            command = wall_following(scan, pid_lidar, pid_imu, SERVO_CENTER, MAX_ADJ, imu_data)

            # If no valid steering command could be calculated, stop the robot
            if not command:
                command = "S\n"

            safe_write(arduino, command)

    except KeyboardInterrupt:
        print("Stopping script...")
    except RPLidarException as e:
        print(f"LiDAR error: {e}")
    finally:
        if lidar:
            lidar.stop()
            lidar.stop_motor()
            lidar.disconnect()
        if arduino and arduino.is_open:
            safe_write(arduino, "S\n")
            arduino.close()
        print("Cleanup done.")

if __name__ == "__main__":
    main()
