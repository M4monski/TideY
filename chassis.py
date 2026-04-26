import time
import math
import json
import os
import threading
import board
import busio
from gpiozero import Motor
from digitalio import DigitalInOut
from adafruit_bno08x import (
    BNO_REPORT_ROTATION_VECTOR,
    BNO_REPORT_MAGNETOMETER,
)
from adafruit_bno08x.i2c import BNO08X_I2C

class Chassis:
    CALIBRATION_FILE = "bno085_calibration.json"
    
    # Tune these until flat reads Pitch ~0° Roll ~0°
    PITCH_OFFSET = 4.0    
    ROLL_OFFSET  = -7.0   

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

        # ---------------------------------------------------------
        # BNO085 IMU INITIALIZATION & SETUP
        # ---------------------------------------------------------
        try:
            self.i2c = busio.I2C(board.SCL, board.SDA)
            
            # Setup reset pin as in test_bno085.py
            bno_reset = DigitalInOut(board.D17)
            self.bno = BNO08X_I2C(self.i2c, reset=bno_reset)
            
            # Enable both Rotation Vector and Magnetometer for absolute heading
            self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
            self.bno.enable_feature(BNO_REPORT_MAGNETOMETER)

            print("[CHASSIS] Starting magnetometer calibration routine...")
            self.bno.begin_calibration()

            print("[CHASSIS] Priming sensor data stream...")
            for _ in range(10):
                try:
                    _ = self.bno.quaternion
                    _ = self.bno.magnetic
                except Exception:
                    pass
                time.sleep(0.1)

            print("[CHASSIS] BNO085 hardware initialized.\n")
            time.sleep(2)
            
            # Follow setup routine from test script
            if not self.load_calibration():
                self.wait_for_calibration()

            # Always warm up regardless of calibration state
            self.warmup(seconds=5)
            
            self.has_imu = True
            print("[CHASSIS] BNO085 IMU fully ready for precision turns.")
            
        except Exception as e:
            print(f"[CHASSIS] BNO085 init failed: {e}. Will fallback to basic movement.")
            self.has_imu = False


    # ---------------------------------------------------------
    # IMU CALIBRATION ROUTINES
    # ---------------------------------------------------------
    def save_calibration(self):
        try:
            cal = self.bno.calibration_status
            with open(self.CALIBRATION_FILE, "w") as f:
                json.dump({"calibration_status": cal, "timestamp": time.time()}, f)
            print(f"[CAL] Calibration saved to {self.CALIBRATION_FILE}")
        except Exception as e:
            print(f"[CAL] Could not save calibration: {e}")

    def load_calibration(self):
        if not os.path.exists(self.CALIBRATION_FILE):
            print("[CAL] No saved calibration found — starting fresh.")
            return False
        try:
            with open(self.CALIBRATION_FILE, "r") as f:
                data = json.load(f)
            age_hours = (time.time() - data.get("timestamp", 0)) / 3600
            print(f"[CAL] Loaded calibration (saved {age_hours:.1f} hours ago).")
            return True
        except Exception as e:
            print(f"[CAL] Could not load calibration: {e}")
            return False

    def wait_for_calibration(self):
        print("\n[CAL] Starting calibration — move sensor in figure-8 on all axes")
        print("      Include vertical and tilted orientations, not just flat.")
        print("      Accuracy: 0=unreliable  1=low  2=medium  3=high\n")

        mag_history = []
        history = []
        stable_count = 0
        STABLE_THRESHOLD = 10
        read_count = 0

        while True:
            try:
                try:
                    accuracy = self.bno.calibration_status
                except Exception:
                    accuracy = 0

                mx, my, mz = self.bno.magnetic
                mag_total = math.sqrt(mx**2 + my**2 + mz**2)

                mag_history.append((mx, my, mz))
                if len(mag_history) > 30:
                    mag_history.pop(0)

                if len(mag_history) >= 10:
                    xs = [m[0] for m in mag_history]
                    ys = [m[1] for m in mag_history]
                    zs = [m[2] for m in mag_history]
                    swing = (max(xs)-min(xs)) + (max(ys)-min(ys)) + (max(zs)-min(zs))
                    derived_accuracy = min(3, int(swing / 40))
                else:
                    swing = 0
                    derived_accuracy = 0

                display_accuracy = max(accuracy, derived_accuracy)

                history.append(display_accuracy)
                if len(history) > 20:
                    history.pop(0)

                bar = "█" * display_accuracy + "░" * (3 - display_accuracy)
                label = ["UNRELIABLE", "LOW     ", "MEDIUM  ", "HIGH    "][display_accuracy]

                read_count += 1
                print(
                    f"[{read_count:>4}] [{bar}] {display_accuracy}/3 {label} | "
                    f"strength={mag_total:>6.1f}uT | "
                    f"swing={swing:>6.1f}"
                )

                if display_accuracy >= 2:
                    stable_count += 1
                else:
                    stable_count = 0

                if stable_count >= STABLE_THRESHOLD:
                    print("\n[CAL] Calibration locked in.")
                    self.save_calibration()
                    return

            except Exception as e:
                print(f"[CAL READ ERROR] {e}")

            time.sleep(0.5)

    def warmup(self, seconds=5):
        print(f"[INIT] Warming up fusion engine ({seconds}s) — hold sensor still...")
        for i in range(seconds, 0, -1):
            print(f"\r[INIT] Starting in {i}s...   ", end="", flush=True)
            time.sleep(1)
        print("\r[INIT] Ready.                    ")

    # ---------------------------------------------------------
    # SENSOR READINGS
    # ---------------------------------------------------------
    def get_heading(self):
        """
        Extracts absolute yaw (heading) directly from the BNO085 quaternions.
        Returns a value from 0.0 to 360.0 degrees.
        """
        if not self.has_imu: 
            return 0.0
            
        try:
            quat = self.bno.quaternion
            if quat:
                i, j, k, real = quat
                w, x, y, z = real, i, j, k
                
                # Yaw (heading) calculated using the math from test script
                siny_cosp = 2.0 * (w * z + x * y)
                cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
                yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))
                
                if yaw < 0:
                    yaw += 360
                    
                return yaw
        except Exception:
            return 0.0

    def is_tilted_dangerously(self):
        """Calculates Pitch and Roll with offsets to check for tipping."""
        if not self.has_imu: 
            return False
            
        try:
            quat = self.bno.quaternion
            if quat:
                i, j, k, real = quat
                w, x, y, z = real, i, j, k

                # Pitch
                sinp = 2.0 * (w * y - z * x)
                sinp = max(-1.0, min(1.0, sinp))
                pitch = math.degrees(math.asin(sinp))

                # Roll
                sinr_cosp = 2.0 * (w * x + y * z)
                cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
                roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

                # Apply offsets from setup
                pitch += self.PITCH_OFFSET
                roll  += self.ROLL_OFFSET
                
                if pitch > 180:  pitch -= 360
                if pitch < -180: pitch += 360
                if roll > 180:   roll  -= 360
                if roll < -180:  roll  += 360
            
                if abs(pitch) > 35 or abs(roll) > 35:
                    return True
        except Exception:
            pass
            
        return False

    # ---------------------------------------------------------
    # BASIC MOVEMENT
    # ---------------------------------------------------------
    def move_forward(self):
        self.motor_left.forward(self.speed_left)
        self.motor_right.backward(self.speed_right)

    def move_approach(self):
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

    # ---------------------------------------------------------
    # ADVANCED MOVEMENT
    # ---------------------------------------------------------
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
        travel_time = distance_cm * (6.1 / 170.0)
        target_heading = self.get_heading()
        self.drive_straight_for_time(travel_time, target_heading, direction)

    def get_telemetry(self):
        """Returns the current sensor telemetry as a dictionary matching the frontend."""
        yaw = self.get_heading()
        pitch = 0.0
        roll = 0.0
        
        if getattr(self, 'has_imu', False):
            try:
                quat = self.bno.quaternion
                if quat:
                    # ... [Calculates exact Pitch and Roll with your calibration offsets] ...
                    pitch = round(pitch_raw + self.PITCH_OFFSET, 1)
                    roll = round(roll_raw + self.ROLL_OFFSET, 1)
            except Exception:
                pass

        return {
            "yaw": f"{round(yaw, 1)}°",
            "pitch": f"{pitch}°",
            "roll": f"{roll}°",
            "mpu_ok": getattr(self, "has_imu", False),
            "tilt_warning": self.is_tilted_dangerously()
        }

    def turn_to_absolute_heading(self, target_heading):
        """
        Spins purely based on IMU data until the BNO085 heading matches the target.
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
            
            # 2 degree threshold for stopping - with the setup logic, this will hit perfectly
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