import serial
from rplidar import RPLidar
import time

# --- Kalman Filter Class ---
class KalmanFilter:
    """A Kalman Filter for fusing IMU data (gyro and magnetometer)."""
    def __init__(self, Q_angle, Q_bias, R_measure):
        self.Q_angle = Q_angle
        self.Q_bias = Q_bias
        self.R_measure = R_measure
        self.angle = 0.0
        self.bias = 0.0
        self.P = [[0.0, 0.0], [0.0, 0.0]]
        self.last_time = time.time() # Initialize last_time to avoid errors

    def filter(self, new_angle, new_rate, dt):
        """Updates the filter with new sensor data."""
        # Step 1: Prediction
        self.angle += dt * (new_rate - self.bias)
        self.P[0][0] += dt * (dt * self.P[1][1] - self.P[0][1] - self.P[1][0] + self.Q_angle)
        self.P[0][1] -= dt * self.P[1][1]
        self.P[1][0] -= dt * self.P[1][1]
        self.P[1][1] += self.Q_bias * dt
        
        # Step 2: Update (Correction)
        S = self.P[0][0] + self.R_measure
        K = [self.P[0][0] / S, self.P[1][0] / S]
        y = new_angle - self.angle
        
        self.angle += K[0] * y
        self.bias += K[1] * y
        
        P00_temp = self.P[0][0]
        P01_temp = self.P[0][1]
        
        self.P[0][0] -= K[0] * P00_temp
        self.P[0][1] -= K[0] * P01_temp
        self.P[1][0] -= K[1] * P00_temp
        # Corrected typo: P11_temp was not defined
        self.P[1][1] -= K[1] * P01_temp

        return self.angle

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
        """Calculates the PID output based on the current error."""
        proportional = self.kp * error
        self.integral += error
        self.integral = max(-self.integral_limit, min(self.integral, self.integral_limit))
        integral_term = self.ki * self.integral
        derivative = self.kd * (error - self.last_error)
        self.last_error = error
        return proportional + integral_term + derivative

# --- Sensor Data Parsing ---
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

# --- Main Script ---
def main():
    lidar = None
    arduino_serial = None
    
    # Initialize PID controller parameters
    KP = 1.5
    KI = 0.01
    KD = 0.5
    SERVO_CENTER_ANGLE = 110
    SERVO_MAX_ADJUSTMENT = 30 # SETTING MAX STEERING DIFFERENCE TO 30 DEGREES
    
    # Initialize controllers
    pid_controller = PIDController(kp=KP, ki=KI, kd=KD)
    kalman_filter = KalmanFilter(Q_angle=0.001, Q_bias=0.003, R_measure=0.03)
    
    # Lap counting variables
    lap_count = 0
    turn_count = 0
    last_fused_heading = 0.0
    turn_threshold = 45 # Degrees change to register a turn

    # Serial port configuration
    # NOTE: You may need to swap these ports depending on your setup
    ARDUINO_SERIAL_PORT = '/dev/ttyUSB1'
    ARDUINO_BAUDRATE = 9600
    LIDAR_PORT = '/dev/ttyUSB0'
    LIDAR_BAUDRATE = 460800 # Corrected to standard baudrate for RPLidar A1/A2

    try:
        # Connect to RPLidar
        print(f"Connecting to RPLIDAR on {LIDAR_PORT}...")
        lidar = RPLidar(LIDAR_PORT, baudrate=LIDAR_BAUDRATE)
        lidar.get_info()

        # Connect to Arduino
        print(f"Connecting to Arduino on {ARDUINO_SERIAL_PORT}...")
        arduino_serial = serial.Serial(ARDUINO_SERIAL_PORT, ARDUINO_BAUDRATE, timeout=1)
        time.sleep(2)

        # Main loop
        print("Starting combined control loop...")
        for scan in lidar.iter_scans():
            # Check for lap completion
            if lap_count >= 3:
                print("Three laps completed! Stopping the robot.")
                arduino_serial.write("S\n".encode('utf-8'))
                break # Exit the main loop

            # Get current time for dt calculation
            current_time = time.time()
            dt = current_time - kalman_filter.last_time
            kalman_filter.last_time = current_time

            # Read IMU data from Arduino
            imu_data = {}
            if arduino_serial.in_waiting > 0:
                line = arduino_serial.readline().decode('utf-8').strip()
                if line:
                    imu_data = parse_sensor_data(line)
            
            # Read LiDAR data
            left_distances = []
            right_distances = []
            
            for quality, angle, distance_mm in scan:
                # Filter for wall-following
                if 60 <= angle <= 90 and distance_mm > 100:
                    left_distances.append(distance_mm / 1000.0)
                if 270 <= angle <= 300 and distance_mm > 100:
                    right_distances.append(distance_mm / 1000.0)

            # Control Logic
            command_to_send = ""
            if left_distances and right_distances:
                # Use PID for wall-following
                avg_left = sum(left_distances) / len(left_distances)
                avg_right = sum(right_distances) / len(right_distances)
                error = avg_left - avg_right
                steering_adjustment = pid_controller.calculate_output(error)
                
                # Map PID output to a servo angle
                clamped_pid_output = max(-1.0, min(steering_adjustment, 1.0))
                final_angle = SERVO_CENTER_ANGLE + (clamped_pid_output * SERVO_MAX_ADJUSTMENT)
                final_angle = max(80, min(140, final_angle))
                command_to_send = f"A{int(final_angle)}\n"
                print(f"Wall-following. Error: {error:.2f}, Steer: {steering_adjustment:.2f}, Angle: {final_angle:.0f}")
            elif 'H' in imu_data and 'GY' in imu_data:
                # Fallback to Kalman filter for straight-line navigation
                imu_data['H'] = 360 - imu_data['H']
                fused_heading = kalman_filter.filter(imu_data['H'], imu_data['GY'], dt)
                
                # Turn detection
                heading_change = abs(fused_heading - last_fused_heading)
                if heading_change > turn_threshold:
                    turn_count += 1
                    last_fused_heading = fused_heading
                    print(f"Turn detected! Turn count: {turn_count}")

                if turn_count >= 4:
                    lap_count += 1
                    turn_count = 0
                    print(f"Lap {lap_count} completed!")

                # Maintain straight line
                target_heading = 0.0 # Maintain initial heading
                error = target_heading - fused_heading
                
                if error > 180: error -= 360
                elif error < -180: error += 360

                steering_adjustment = error * 0.5 # Simple P-controller
                
                final_angle = SERVO_CENTER_ANGLE + steering_adjustment
                final_angle = max(80, min(140, final_angle))
                command_to_send = f"A{int(final_angle)}\n"
                print(f"IMU fallback. Fused H: {fused_heading:.2f}, Steer: {steering_adjustment:.2f}, Angle: {final_angle:.0f}")
            else:
                command_to_send = "S\n"
                print("No data. Sending STOP command.")

            # Send command to Arduino
            if command_to_send:
                arduino_serial.write(command_to_send.encode('utf-8'))

    except KeyboardInterrupt:
        print("Stopping script...")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        if lidar:
            lidar.stop()
            lidar.disconnect()
        if arduino_serial:
            arduino_serial.write("S\n".encode('utf-8'))
            arduino_serial.close()

if __name__ == '__main__':
    main()
