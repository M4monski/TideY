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
        
        self.base_speed = config.get("speed", 0.7)
        self.turn_speed = config.get("turn_speed", 0.8)
        
        self.left_trim = 1.0  
        self.right_trim = 1.0  
        
        self.speed_left = self.base_speed * self.left_trim
        self.speed_right = self.base_speed * self.right_trim

        self.vision = None
        
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
    # SENSOR READINGS W/ GLITCH PROTECTION
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
        except Exception as e:
            # We catch hardware I2C glitches here and return None instead of 0.0
            print(f"[IMU WARNING] Sensor glitch: {e}")
            return None

    def get_heading_smoothed(self, samples=7):
        readings = []
        for _ in range(samples):
            h = self.get_heading()
            if h is not None:
                readings.append(h)
            time.sleep(0.010)

        # If all samples glitched out, return None
        if not readings:
            return None

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
            diff = abs((r - mean_deg + 540) % 360 - 180)
            if diff <= 20.0:
                filtered_sin.append(math.sin(math.radians(r)))
                filtered_cos.append(math.cos(math.radians(r)))

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
    # ADVANCED MOVEMENT & AUTO-PICKUP
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
                
            # --- RED TAPE OVERRIDE ---
            if self.vision and self.vision.red_tape_triggered:
                print("\n[CHASSIS] --- RED TAPE BOUNDARY HIT --- Turning around early!\n")
                self.vision.red_tape_triggered = False  
                self.stop()
                return 
                
            # --- AUTO PICKUP OVERRIDE ---
            if self.vision and getattr(self.vision, 'target_in_response_zone', False):
                self.stop()
                
                # Capture the exact trash type before we start maneuvering
                trash_type = getattr(self.vision, 'target_class', "Unknown")
                
                time_driven_so_far = time.time() - start_time
                
                # Go do the complex pickup, sorting, and return maneuver
                self.execute_pickup_and_return(trash_type)
                
                # We are back on the line! Deduct the time we already drove 
                travel_time = travel_time - time_driven_so_far
                start_time = time.time()
                print(f"[CHASSIS] Resuming sweep lane. {travel_time:.2f}s remaining.")
                
            current_heading = self.get_heading()
            
            # Glitch skip: keep moving if sensor drops out momentarily
            if current_heading is None:
                time.sleep(0.01)
                continue
                
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

    def execute_pickup_and_return(self, trash_type="Unknown"):
        print(f"\n[PICKUP] --- INITIATING AUTO-PICKUP for: {trash_type.upper()} ---")
        
        # 1. SAVE THE ORIGINAL SWEEP STATE
        original_heading = self.get_heading_smoothed(samples=5)
        if original_heading is None:
            original_heading = self.get_heading() or 0.0
        print(f"[PICKUP] Saved original sweep line heading: {original_heading:.1f}°")

        # Blindfold vision tape detection
        if self.vision:
            self.vision.pause_tape_detection()

        # 2. ALIGN TO TRASH
        print("[PICKUP] Aligning to target...")
        align_timeout = time.time() + 5.0
        while time.time() < align_timeout:
            if not self.vision or self.vision.target_x is None:
                break
            
            error_x = self.vision.target_x - 320 
            if abs(error_x) < 25: 
                self.stop()
                break 
                
            align_speed = 0.32 
            if error_x > 0:
                self.motor_left.forward(align_speed)
                self.motor_right.forward(align_speed)
            else:
                self.motor_left.backward(align_speed)
                self.motor_right.backward(align_speed)
            time.sleep(0.02)
        self.stop()

        # 3. APPROACH AND RECORD TIME
        print("[PICKUP] Approaching target...")
        approach_start = time.time()
        approach_timeout = approach_start + 6.0 
        
        while time.time() < approach_timeout:
            if self.vision and self.vision.target_in_grab_zone:
                break 
            self.move_approach()
            time.sleep(0.02)
            
        self.stop()
        approach_time = time.time() - approach_start
        print(f"[PICKUP] Target reached. Forward drive took: {approach_time:.2f}s")

        # 4. EXECUTE GRAB
        print("[PICKUP] *** ACTIVATING ARMS / GRABBING TRASH ***")
        time.sleep(3.0) 
        
        # 5. SORTING DROP-OFF MANEUVER
        trash_lower = trash_type.lower()
        plastic_classes = ["general_plastic", "plastic_bottles", "plastic_bottle"]
        glass_classes = ["glass_bottle", "glass"]
        
        approach_heading = self.get_heading_smoothed(samples=3)
        if approach_heading is None: 
            approach_heading = self.get_heading() or original_heading
            
        if trash_lower in plastic_classes:
            print(f"[SORTING] Plastic ({trash_type}) detected! Turning 90° RIGHT to drop...")
            drop_heading = (approach_heading - 90) % 360
            self.turn_to_absolute_heading(drop_heading)
            
            print("[SORTING] Releasing Plastic...")
            time.sleep(2.0)
            
            print("[SORTING] Re-aligning to reverse line...")
            self.turn_to_absolute_heading(approach_heading)

        elif trash_lower in glass_classes:
            print(f"[SORTING] Glass ({trash_type}) detected! Turning 90° LEFT to drop...")
            drop_heading = (approach_heading + 90) % 360
            self.turn_to_absolute_heading(drop_heading)
            
            print("[SORTING] Releasing Glass...")
            time.sleep(2.0)
            
            print("[SORTING] Re-aligning to reverse line...")
            self.turn_to_absolute_heading(approach_heading)
            
        else:
            print(f"[SORTING] Unknown/Unsorted item '{trash_type}'. Holding in main carriage.")

        # 6. REVERSE EXACT DEVIATION
        print(f"[PICKUP] Reversing back to sweep line for {approach_time:.2f}s...")
        reverse_start = time.time()
        
        approach_factor = 0.65 
        l_speed = max(0.35, self.speed_left * approach_factor)
        r_speed = max(0.35, self.speed_right * approach_factor)
        
        while time.time() - reverse_start < approach_time:
            self.motor_left.backward(l_speed)
            self.motor_right.forward(r_speed)
            time.sleep(0.02)
        self.stop()

        # 7. RESTORE ORIGINAL HEADING
        print("[PICKUP] Snapping back to original sweep heading...")
        self.turn_to_absolute_heading(original_heading)

        if self.vision:
            self.vision.resume_tape_detection()
            
        print("[PICKUP] --- SEQUENCE COMPLETE. RESUMING SWEEP ---\n")

    def get_telemetry(self):
        yaw = self.get_heading()
        pitch = 0.0
        roll = 0.0
        
        # Safely handle the string conversion if I2C glitches during a web request
        if yaw is not None:
            yaw_str = f"{round(yaw, 1)}"
        else:
            yaw_str = "ERR"

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
            "yaw": yaw_str,
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
        MIN_TURN_SPEED = 0.35 # Motor floor to prevent stall
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
            
            # Glitch Skip
            if current is None:
                time.sleep(0.01)
                continue
                
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
                self.motor_left.backward(speed)
                self.motor_right.backward(speed)
            else:
                self.motor_left.forward(speed)
                self.motor_right.forward(speed)

            time.sleep(0.01)

        final_heading = self.get_heading_smoothed()
        if final_heading is not None:
            print(f"[CHASSIS] Turn complete. Final Heading: {final_heading:.2f}\n")

    def arc_turn_to_heading(self, target_heading, turn_deg):
        target_heading = target_heading % 360

        OUTER_SPEED = self.base_speed
        INNER_SPEED = self.base_speed * 0.4 

        SLOW_ZONE      = 30.0 
        MIN_SPEED_MULT = 0.45 

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
            
            # Glitch Skip
            if current is None:
                time.sleep(0.01)
                continue
                
            error = (target_heading - current + 540) % 360 - 180

            if abs(error) < DEADBAND:
                self.stop()
                time.sleep(0.05)
                break

            t = min(abs(error) / SLOW_ZONE, 1.0)
            speed_mult = MIN_SPEED_MULT + t * (1.0 - MIN_SPEED_MULT)

            raw_outer = OUTER_SPEED * speed_mult
            raw_inner = INNER_SPEED * speed_mult
            
            # Absolute motor floor to prevent Errno 121 stalls
            outer = max(0.35, raw_outer)
            inner = max(0.35, raw_inner)

            if turn_deg < 0:
                self.motor_left.forward(outer * self.left_trim)
                self.motor_right.backward(inner * self.right_trim)
            else:
                self.motor_left.forward(inner * self.left_trim)
                self.motor_right.backward(outer * self.right_trim)

            time.sleep(0.02)

        final = self.get_heading_smoothed()
        if final is not None:
            print(f"[CHASSIS] Arc turn complete. Final heading: {final:.1f}°")
        return final

    def sweep_area(self, grid_size_cm):
        lane_width_cm = 50.0
        rest_time     = 0.5

        lanes     = max(1, int(grid_size_cm / lane_width_cm))
        lane_time = grid_size_cm * (6.1 / 170.0)

        current_heading = self.get_heading_smoothed(samples=10)
        if current_heading is None:
            current_heading = self.get_heading() or 0.0
            
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
            
            # --- BLINDFOLD THE VISION SYSTEM ---
            if self.vision:
                self.vision.pause_tape_detection()
                
            self.arc_turn_to_heading((current_heading + turn_deg) % 360, turn_deg)

            self.stop()
            time.sleep(1.0)
            
            # --- OPEN EYES FOR THE NEXT LANE ---
            if self.vision:
                self.vision.resume_tape_detection()
                
            new_heading = self.get_heading_smoothed(samples=10)
            if new_heading is not None:
                current_heading = new_heading
                
            print(f"[SWEEP] Fresh IMU heading after arc: {current_heading:.1f}° — this is the new straight")

            time.sleep(0.3)
            use_right_turn = not use_right_turn

        print("[SWEEP] --- SWEEP COMPLETE ---\n")