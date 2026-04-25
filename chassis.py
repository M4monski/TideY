import time
import math
import threading
import board
import busio
from gpiozero import Motor
from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR
from adafruit_bno08x.i2c import BNO08X_I2C

class Chassis:
    def __init__(self, config):
        """
        Initializes the motor controller and the BNO085 sensor.
        """
        left_pins = config.get("left_pins", [13, 19])
        right_pins = config.get("right_pins", [18, 12])
        
        self.motor_left = Motor(forward=left_pins[0], backward=left_pins[1])
        self.motor_right = Motor(forward=right_pins[0], backward=right_pins[1])
        
        self.base_speed = config.get("speed", 0.5)
        self.turn_speed = config.get("turn_speed", 0.6)
        
        # ---------------------------------------------------------
        # MOTOR TRIM CALIBRATION
        # ---------------------------------------------------------
        self.left_trim = 0.91  
        self.right_trim = 1.0  
        
        self.speed_left = self.base_speed * self.left_trim
        self.speed_right = self.base_speed * self.right_trim
        
        print("[CHASSIS] Motors Initialized.")

        # Initialize BNO085 on the shared I2C bus
        try:
            self.i2c = busio.I2C(board.SCL, board.SDA)
            self.bno = BNO08X_I2C(self.i2c)
            # Enable the Rotation Vector feature (Absolute Orientation)
            self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
            self.has_imu = True
            print("[CHASSIS] BNO085 IMU Initialized successfully.")
            
        except Exception as e:
            print(f"[CHASSIS] BNO085 init failed: {e}. Will fallback to basic movement.")
            self.has_imu = False

    def get_heading(self):
        """
        Extracts absolute yaw (heading) directly from the BNO085 quaternions.
        Returns a value from 0.0 to 360.0 degrees.
        """
        if not self.has_imu: 
            return 0.0
            
        try:
            i, j, k, real = self.bno.quaternion
            # Convert quaternion to yaw (Z-axis rotation)
            siny_cosp = 2.0 * (real * k + i * j)
            cosy_cosp = 1.0 - 2.0 * (j * j + k * k)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            heading = math.degrees(yaw)
            
            # Normalize to 0-360
            if heading < 0:
                heading += 360
            return heading
        except Exception:
            return 0.0

    # --- BASIC MOVEMENT ---
    def move_forward(self):
        self.motor_left.forward(self.speed_left)
        self.motor_right.backward(self.speed_right)

    def move_approach(self):
        """A slower, precision speed to prevent overshooting the target."""
        approach_factor = 0.65 
        l_speed = max(0.35, self.speed_left * approach_factor)
        r_speed = max(0.35, self.speed_right * approach_factor)
        
        self.motor_left.forward(l_speed)
        self.motor_right.backward(r_speed)

    def move_backward(self):
        self.motor_left.backward(self.speed_left)
        self.motor_right.forward(self.speed_right)

    def spin_left(self):
        self.motor_left.backward(self.turn_speed)
        self.motor_right.backward(self.turn_speed)

    def spin_right(self):
        self.motor_left.forward(self.turn_speed)
        self.motor_right.forward(self.turn_speed)

    def stop(self):
        self.motor_left.stop()
        self.motor_right.stop()

    # --- SENSOR SAFETY ---
    def is_tilted_dangerously(self):
        """Calculates Pitch and Roll from BNO085 quaternions to check for tipping."""
        if not self.has_imu: 
            return False
            
        try:
            i, j, k, real = self.bno.quaternion
            # Pitch
            sinp = 2.0 * (real * j - k * i)
            pitch = math.degrees(math.asin(sinp)) if abs(sinp) <= 1 else math.degrees(math.copysign(math.pi / 2, sinp))
            # Roll
            sinr_cosp = 2.0 * (real * i + j * k)
            cosr_cosp = 1.0 - 2.0 * (i * i + j * j)
            roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))
            
            if abs(pitch) > 35 or abs(roll) > 35:
                return True
        except Exception:
            pass
            
        return False

    # --- ADVANCED MOVEMENT ---
    def drive_straight_for_time(self, travel_time, target_heading, direction='w'):
        """
        Actively drives for a set time while using the IMU to lock onto a heading.
        """
        print(f"[CHASSIS] Driving for {travel_time:.2f}s. Locking heading to {target_heading:.1f}°")
        
        if direction == 's':
            self.move_backward()
            time.sleep(travel_time)
            self.stop()
            return 
        
        start_time = time.time()
        while (time.time() - start_time) < travel_time:
            if self.is_tilted_dangerously():
                self.stop()
                print("\n[CHASSIS] 🚨 EMERGENCY STOP: Excessive tilt detected! 🚨\n")
                return 
                
            current_heading = self.get_heading()
            
            # Shortest path error calculation (-180 to +180)
            error = (target_heading - current_heading + 540) % 360 - 180
            
            correction_strength = 0.015 
            correction = error * correction_strength
            
            raw_l = self.speed_left - correction
            raw_r = self.speed_right + correction
            
            min_power = 0.35
            l_speed = max(min_power, min(1.0, raw_l))
            r_speed = max(min_power, min(1.0, raw_r))
            
            self.motor_left.forward(l_speed)
            self.motor_right.backward(r_speed)
            
            time.sleep(0.02) 
            
        self.stop()

    def move_set_distance(self, distance_cm, direction='w'):
        """Maintains backward compatibility with server distance commands."""
        travel_time = distance_cm * (6.1 / 170.0)
        target_heading = self.get_heading()
        self.drive_straight_for_time(travel_time, target_heading, direction)

    def turn_to_absolute_heading(self, target_heading):
        """
        Spins purely based on IMU data until the BNO085 heading matches the target.
        No timers, no cutoffs.
        """
        if not self.has_imu:
            print("[CHASSIS] No IMU. Cannot execute pure absolute turn.")
            return

        target_heading = target_heading % 360
        print(f"\n[CHASSIS] --- SNAPPING TO GRID HEADING: {target_heading:.2f}° ---")
        
        while True:
            current = self.get_heading()
            # Calculate the shortest path (-180 to 180)
            error = (target_heading - current + 540) % 360 - 180
            
            # 2 degree threshold for stopping
            if abs(error) < 2.0:
                self.stop()
                break
                
            if error > 0:
                self.spin_right()
            else:
                self.spin_left()
                
            time.sleep(0.01)
            
        print(f"[CHASSIS] Turn complete. Final Heading: {self.get_heading():.2f}°\n")

    def sweep_area(self, grid_size_cm):
        """Executes a Boustrophedon sweep relying purely on BNO085 90-deg turns."""
        lane_width = 50.0
        rest_time = 1.0  
        
        lanes = int(grid_size_cm / lane_width)
        if lanes < 1: 
            lanes = 1

        print(f"\n[CHASSIS] --- STARTING SWEEP ---")
        
        # Memorize exact forward travel times as requested
        memorized_lane_time = grid_size_cm * (6.1 / 170.0)
        memorized_width_time = lane_width * (6.1 / 170.0)
        
        # Lock initial heading to base the entire grid off of
        grid_target = self.get_heading() 
        turn_direction = 1 
        
        for i in range(lanes):
            # Move forward using memorized time
            self.drive_straight_for_time(memorized_lane_time, grid_target)
            
            if i == lanes - 1:
                break
                
            time.sleep(rest_time)
                
            # TURN 1 (90 degrees)
            grid_target = (grid_target + (90.0 if turn_direction == 1 else -90.0)) % 360  
            self.turn_to_absolute_heading(grid_target)
            time.sleep(rest_time)
            
            # DRIVE LANE WIDTH using memorized time
            self.drive_straight_for_time(memorized_width_time, grid_target)
            time.sleep(rest_time)
            
            # TURN 2 (90 degrees)
            grid_target = (grid_target + (90.0 if turn_direction == 1 else -90.0)) % 360  
            self.turn_to_absolute_heading(grid_target)
            time.sleep(rest_time)
            
            turn_direction *= -1
            
        print("[CHASSIS] --- SWEEP COMPLETE ---\n")