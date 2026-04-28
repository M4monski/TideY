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
    
    PITCH_OFFSET = 4.0    
    ROLL_OFFSET  = -7.0   

    def __init__(self, config):
        left_pins = config.get("left_pins", [13, 19])
        right_pins = config.get("right_pins", [18, 12])
        
        self.motor_left = Motor(forward=left_pins[0], backward=left_pins[1])
        self.motor_right = Motor(forward=right_pins[0], backward=right_pins[1])
        
        self.base_speed = config.get("speed", 0.8)
        self.turn_speed = config.get("turn_speed", 0.8)
        
        self.left_trim = 1.0  
        self.right_trim = 1.0  
        
        self.speed_left = self.base_speed * self.left_trim
        self.speed_right = self.base_speed * self.right_trim
        
        print("[CHASSIS] Motors Initialized.")

        try:
            self.i2c = busio.I2C(board.SCL, board.SDA)
            
            bno_reset = DigitalInOut(board.D17)
            self.bno = BNO08X_I2C(self.i2c, reset=bno_reset)
            
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
            
            if not self.load_calibration():
                self.wait_for_calibration()

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
            print("[CAL] No saved calibration found -- starting fresh.")
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
        print("\n[CAL] Starting calibration -- move sensor in figure-8 on all axes")
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

                bar = "X" * display_accuracy + "." * (3 - display_accuracy)
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
        print(f"[INIT] Warming up fusion engine ({seconds}s) -- hold sensor still...")
        for i in range(seconds, 0, -1):
            print(f"\r[INIT] Starting in {i}s...   ", end="", flush=True)
            time.sleep(1)
        print("\r[INIT] Ready.                    ")

    # ---------------------------------------------------------
    # SENSOR READINGS
    # ---------------------------------------------------------
    def get_heading(self):
        if not self.has_imu: 
            return 0.0
            
        try:
            quat = self.bno.quaternion
            if quat:
                i, j, k, real = quat
                w, x, y, z = real, i, j, k
                
                siny_cosp = 2.0 * (w * z + x * y)
                cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
                yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))
                
                if yaw < 0:
                    yaw += 360
                    
                return yaw
        except Exception:
            return 0.0

    def get_heading_smoothed(self, samples=7):
        readings = []
        for _ in range(samples):
            h = self.get_heading()
            readings.append(h)
            time.sleep(0.010)

        # Convert to unit vectors to handle wrap-around properly
        sin_vals = [math.sin(math.radians(r)) for r in readings]
        cos_vals = [math.cos(math.radians(r)) for r in readings]

        # Compute mean vector
        sin_mean = sum(sin_vals) / len(sin_vals)
        cos_mean = sum(cos_vals) / len(cos_vals)
        mean_deg = math.degrees(math.atan2(sin_mean, cos_mean)) % 360

        # Reject samples that deviate more than 20° from the mean
        filtered_sin = []
        filtered_cos = []
        for r in readings:
            # Angular distance from mean, wrap-safe
            diff = abs((r - mean_deg + 540) % 360 - 180)
            if diff <= 20.0:
                filtered_sin.append(math.sin(math.radians(r)))
                filtered_cos.append(math.cos(math.radians(r)))

        # Fall back to full set if all got rejected (shouldn't happen)
        if not filtered_sin:
            filtered_sin, filtered_cos = sin_vals, cos_vals

        avg = math.degrees(math.atan2(
            sum(filtered_sin) / len(filtered_sin),
            sum(filtered_cos) / len(filtered_cos)
        )) % 360

        return avg

    def is_tilted_dangerously(self):
        if not self.has_imu: 
            return False
            
        try:
            quat = self.bno.quaternion
            if quat:
                i, j, k, real = quat
                w, x, y, z = real, i, j, k

                sinp = 2.0 * (w * y - z * x)
                sinp = max(-1.0, min(1.0, sinp))
                pitch = math.degrees(math.asin(sinp))

                sinr_cosp = 2.0 * (w * x + y * z)
                cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
                roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

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
        print(f"[CHASSIS] Driving for {travel_time:.2f}s. Locking heading to {target_heading:.1f}")
        
        if direction == 's':
            self.move_backward()
            time.sleep(travel_time)
            self.stop()
            return 
        
        start_time = time.time()
        while (time.time() - start_time) < travel_time:
            if self.is_tilted_dangerously():
                self.stop()
                print("\n[CHASSIS] EMERGENCY STOP: Excessive tilt detected!\n")
                return 
                
            current_heading = self.get_heading()
            error = (target_heading - current_heading + 540) % 360 - 180
            
            if abs(error) <= 2.0:
                error = 0.0
            
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
        yaw = self.get_heading()
        pitch = 0.0
        roll = 0.0
        
        if getattr(self, 'has_imu', False):
            try:
                quat = self.bno.quaternion
                if quat:
                    i, j, k, real = quat
                    w, x, y, z = real, i, j, k
                    
                    sinp = 2.0 * (w * y - z * x)
                    sinp = max(-1.0, min(1.0, sinp))
                    pitch_raw = math.degrees(math.asin(sinp))

                    sinr_cosp = 2.0 * (w * x + y * z)
                    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
                    roll_raw = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

                    pitch = round(pitch_raw + self.PITCH_OFFSET, 1)
                    roll = round(roll_raw + self.ROLL_OFFSET, 1)
            except Exception:
                pass

        return {
            "yaw": f"{round(yaw, 1)}",
            "pitch": f"{pitch}",
            "roll": f"{roll}",
            "mpu_ok": getattr(self, "has_imu", False),
            "tilt_warning": self.is_tilted_dangerously()
        }

    def turn_to_absolute_heading(self, target_heading):
        if not self.has_imu:
            print("[CHASSIS] No IMU. Cannot execute pure absolute turn.")
            return

        target_heading = target_heading % 360
        print(f"\n[CHASSIS] --- SNAPPING TO GRID HEADING: {target_heading:.2f} ---")

        DEADBAND       = 4.0
        SLOW_ZONE      = 25.0
        MIN_TURN_SPEED = 0.28
        MAX_TURN_SPEED = self.turn_speed
        SETTLE_READS   = 6
        TIMEOUT        = 10.0

        settled = 0
        start_time = time.time()

        while True:
            if time.time() - start_time > TIMEOUT:
                self.stop()
                print("[CHASSIS] Turn timeout -- stopping.")
                break

            current = self.get_heading_smoothed(samples=5)
            error = (target_heading - current + 540) % 360 - 180

            if abs(error) < DEADBAND:
                self.stop()
                time.sleep(0.05)
                settled += 1
                if settled >= SETTLE_READS:
                    break
                time.sleep(0.02)
                continue
            else:
                settled = 0

            t = min(abs(error) / SLOW_ZONE, 1.0)
            speed = MIN_TURN_SPEED + t * (MAX_TURN_SPEED - MIN_TURN_SPEED)

            if error > 0:
                # Target is to the LEFT, so spin LEFT
                self.motor_left.backward(speed)
                self.motor_right.backward(speed)
            else:
                # Target is to the RIGHT, so spin RIGHT
                self.motor_left.forward(speed)
                self.motor_right.forward(speed)

            time.sleep(0.01)

        print(f"[CHASSIS] Turn complete. Final Heading: {self.get_heading_smoothed():.2f}\n")

    def arc_turn_to_heading(self, target_heading, turn_deg):
        """
        Turns to target_heading using differential steering.
        Both motors stay moving — outer wheel faster, inner wheel slower.

        Includes a slow zone that ramps both motor speeds down as the robot
        approaches the target heading, preventing overshoot on loose terrain.

        Args:
            target_heading: absolute heading to reach (degrees)
            turn_deg:       negative = right arc, positive = left arc
        Returns:
            final heading after arc completes
        """
        target_heading = target_heading % 360

        OUTER_SPEED = self.base_speed
        INNER_SPEED = self.base_speed * 0.35  # tune to match lane width

        # Speed ramp — same concept as turn_to_absolute_heading
        SLOW_ZONE      = 30.0  # degrees — start slowing down when this close to target
        MIN_SPEED_MULT = 0.45  # minimum multiplier applied to both wheels in slow zone
                            # keeps robot moving while reducing overshoot risk

        DEADBAND = 5.0
        TIMEOUT  = 20.0

        print(f"[CHASSIS] Arc U-turn -> {target_heading:.1f}° ({'RIGHT' if turn_deg < 0 else 'LEFT'})")

        start_time = time.time()

        while True:
            if time.time() - start_time > TIMEOUT:
                self.stop()
                print("[CHASSIS] Arc turn timeout -- stopping.")
                break

            current = self.get_heading_smoothed(samples=3)
            error   = (target_heading - current + 540) % 360 - 180

            if abs(error) < DEADBAND:
                self.stop()
                time.sleep(0.05)
                break

            # Ramp both wheels down proportionally as we approach target.
            # The outer/inner ratio stays the same so arc radius is preserved.
            t = min(abs(error) / SLOW_ZONE, 1.0)
            speed_mult = MIN_SPEED_MULT + t * (1.0 - MIN_SPEED_MULT)

            outer = OUTER_SPEED * speed_mult
            inner = INNER_SPEED * speed_mult

            if turn_deg < 0:
                # RIGHT arc: left is outer (faster), right is inner (slower)
                self.motor_left.forward(outer * self.left_trim)
                self.motor_right.backward(inner * self.right_trim)
            else:
                # LEFT arc: right is outer (faster), left is inner (slower)
                self.motor_left.forward(inner * self.left_trim)
                self.motor_right.backward(outer * self.right_trim)

            time.sleep(0.02)

        final = self.get_heading_smoothed()
        print(f"[CHASSIS] Arc turn complete. Final heading: {final:.1f}°")
        return final

    def sweep_area(self, grid_size_cm):
        lane_width_cm = 50.0
        rest_time     = 0.5

        lanes     = max(1, int(grid_size_cm / lane_width_cm))
        lane_time = grid_size_cm * (6.1 / 170.0)

        # Get the very first heading and drive the first lane with it
        current_heading = self.get_heading_smoothed(samples=10)
        print(f"\n[SWEEP] Starting sweep: {lanes} lanes")
        print(f"[SWEEP] Initial heading = {current_heading:.1f}°")

        use_right_turn = True

        for i in range(lanes):
            print(f"[SWEEP] Lane {i + 1}/{lanes} | heading = {current_heading:.1f}°")
            self.drive_straight_for_time(lane_time, current_heading)

            if i == lanes - 1:
                break

            time.sleep(rest_time)

            turn_deg = -180.0 if use_right_turn else +180.0
            label    = "RIGHT" if use_right_turn else "LEFT"
            print(f"[SWEEP] 180° {label} arc U-turn")
            self.arc_turn_to_heading((current_heading + turn_deg) % 360, turn_deg)

            # Stop, let IMU fully settle, read fresh — this becomes the new truth
            self.stop()
            time.sleep(1.0)
            current_heading = self.get_heading_smoothed(samples=10)
            print(f"[SWEEP] Fresh IMU heading after arc: {current_heading:.1f}° — this is the new straight")

            time.sleep(0.3)
            use_right_turn = not use_right_turn

        print("[SWEEP] --- SWEEP COMPLETE ---\n")