import time
import math
import threading
import board
import busio
import adafruit_mpu6050
from gpiozero import Motor

class Chassis:
    def __init__(self, config):
        """
        Initializes the motor controller and the MPU6050 sensor.
        """
        left_pins = config.get("left_pins", [13, 19])
        right_pins = config.get("right_pins", [18, 12])
        
        self.motor_left = Motor(forward=left_pins[0], backward=left_pins[1])
        self.motor_right = Motor(forward=right_pins[0], backward=right_pins[1])
        
        self.base_speed = config.get("speed", 0.5)
        self.turn_speed = config.get("turn_speed", 0.6)
        
        # ---------------------------------------------------------
        # MOTOR TRIM CALIBRATION
        # Drifting Right? Lower the left_trim (e.g., 0.92)
        # Drifting Left? Lower the right_trim (e.g., 0.92)
        # ---------------------------------------------------------
        self.left_trim = 0.91  # Slows the left motor down to 92% of its normal speed
        self.right_trim = 1.0  # Keeps the right motor at 100% of its normal speed
        
        self.speed_left = self.base_speed * self.left_trim
        self.speed_right = self.base_speed * self.right_trim
        
        # --- ABSOLUTE TRACKING SYSTEM ---
        self.global_yaw = 0.0  # Keeps track of absolute heading forever
        
        print("[CHASSIS] Motors Initialized.")

        # Initialize MPU6050 on the shared I2C bus
        try:
            self.i2c = busio.I2C(board.SCL, board.SDA)
            self.mpu = adafruit_mpu6050.MPU6050(self.i2c)
            self.has_mpu = True
            print("[CHASSIS] MPU6050 IMU Initialized successfully.")
            
            # --- GYRO CALIBRATION ---
            print("[CHASSIS] >>> STARTING CALIBRATION. KEEP ROBOT FLAT AND STILL...")
            gyro_samples = []
            for _ in range(200):
                gyro_samples.append(self.mpu.gyro[2])
                time.sleep(0.005)
            
            self.gyro_z_bias = sum(gyro_samples) / len(gyro_samples)
            print(f"[CHASSIS] >>> CALIBRATION COMPLETE. Z-Bias: {self.gyro_z_bias:.4f} rad/s")
            
            # Start the background thread to track heading continuously
            threading.Thread(target=self._imu_tracker, daemon=True).start()
            
        except Exception as e:
            print(f"[CHASSIS] MPU6050 init failed: {e}. Will fallback to time-based turning.")
            self.has_mpu = False

    def _imu_tracker(self):
        """
        Runs continuously in the background. Tracks all rotations (turns, drifts).
        Noise filter lowered back to 0.5 so we don't miss slow sand drift!
        """
        last_time = time.time()
        while getattr(self, 'has_mpu', False):
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time
            
            try:
                raw_z_rads = self.mpu.gyro[2]
                corrected_z_rads = raw_z_rads - self.gyro_z_bias
                z_degs = math.degrees(corrected_z_rads)
                
                if abs(z_degs) > 0.5:
                    self.global_yaw += (z_degs * dt)
                    
            except Exception:
                pass
                
            time.sleep(0.01)  # Run at 100Hz

    # --- BASIC MOVEMENT ---
    def move_forward(self):
        self.motor_left.forward(self.speed_left)
        self.motor_right.backward(self.speed_right)

    def move_approach(self):
        """A slower, precision speed to prevent overshooting the target."""
        approach_factor = 0.65  # Drops speed to 65% of normal
        
        # We still use max(0.35) so it doesn't stall in the sand!
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
        if not self.has_mpu: 
            return False
            
        accel_x, accel_y, accel_z = self.mpu.acceleration
        pitch = math.degrees(math.atan2(accel_x, math.sqrt(accel_y**2 + accel_z**2)))
        roll = math.degrees(math.atan2(accel_y, math.sqrt(accel_x**2 + accel_z**2)))
        
        if abs(pitch) > 35 or abs(roll) > 35:
            return True
        return False

    # --- ADVANCED MOVEMENT ---
    def move_set_distance(self, distance_cm, direction='w'):
        """
        Actively drives a set distance while using the Gyroscope to lock onto 
        its current heading. Includes Sand-Safe torque clamping!
        """
        travel_time = distance_cm * (6.1 / 170.0)
        
        # We now lock onto EXACTLY what the sensor reads right now
        target_heading = self.global_yaw
            
        print(f"[CHASSIS] Driving {distance_cm}cm. Locking heading to {target_heading:.1f}°")
        
        if direction == 's':
            self.move_backward()
            return 
        
        start_time = time.time()
        while (time.time() - start_time) < travel_time:
            if self.is_tilted_dangerously():
                self.stop()
                print("\n[CHASSIS] 🚨 EMERGENCY STOP: Excessive tilt detected! 🚨\n")
                return 
                
            error = target_heading - self.global_yaw
            
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

    def turn_to_absolute_heading(self, target_heading, direction):
        """
        Spins until the background thread's global_yaw matches the target_heading.
        """
        if not getattr(self, 'has_mpu', False):
            if direction == 'r': self.spin_right()
            else: self.spin_left()
            time.sleep(0.8)
            self.stop()
            return

        print(f"\n[CHASSIS] --- SNAPPING TO GRID HEADING: {target_heading:.2f}° ---")
        
        right_cutoff = 12.0 
        left_cutoff = -20.5  
        
        start_time = time.time()
            
        if direction == 'r':
            self.spin_right()
            while self.global_yaw > (target_heading + right_cutoff):
                time.sleep(0.01)
                if time.time() - start_time > 5.0: break
        else:
            self.spin_left()
            while self.global_yaw < (target_heading - left_cutoff):
                time.sleep(0.01)
                if time.time() - start_time > 5.0: break
            
        self.stop()
        print(f"[CHASSIS] Turn complete. Final Global Yaw: {self.global_yaw:.2f}°\n")

    def turn_90(self, direction='r'):
        if direction == 'r':
            target = self.global_yaw - 90.0
        else:
            target = self.global_yaw + 90.0
        self.turn_to_absolute_heading(target, direction)

    def sweep_area(self, grid_size_cm):
        """Executes a Boustrophedon sweep."""
        lane_width = 50.0
        rest_time = 1.0  
        
        lanes = int(grid_size_cm / lane_width)
        if lanes < 1: 
            lanes = 1

        print(f"\n[CHASSIS] --- STARTING SWEEP ---")
        
        self.global_yaw = 0.0
        grid_target = 0.0 
        turn_direction = 1 
        
        for i in range(lanes):
            self.move_set_distance(grid_size_cm, 'w')
            
            if i == lanes - 1:
                break
                
            time.sleep(rest_time)
                
            if turn_direction == 1:
                grid_target -= 90.0  
                self.turn_to_absolute_heading(grid_target, 'r')
                time.sleep(rest_time)
                self.move_set_distance(lane_width, 'w')
                time.sleep(rest_time)
                grid_target -= 90.0  
                self.turn_to_absolute_heading(grid_target, 'r')
            else:
                grid_target += 90.0  
                self.turn_to_absolute_heading(grid_target, 'l')
                time.sleep(rest_time)
                self.move_set_distance(lane_width, 'w')
                time.sleep(rest_time)
                grid_target += 90.0  
                self.turn_to_absolute_heading(grid_target, 'l')
                
            time.sleep(rest_time)
            turn_direction *= -1
            
        print("[CHASSIS] --- SWEEP COMPLETE ---\n")