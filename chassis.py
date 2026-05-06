import time
import math
import json
import os
import threading
import board
import busio
from gpiozero import Motor
from digitalio import DigitalInOut, Direction, Pull
from adafruit_bno08x import (
    BNO_REPORT_ROTATION_VECTOR,
    BNO_REPORT_MAGNETOMETER,
)
from adafruit_bno08x.i2c import BNO08X_I2C
from ina226 import INA226

class Chassis:
    CALIBRATION_FILE = "bno085_calibration.json"
    
    PITCH_OFFSET = 4.0    
    ROLL_OFFSET  = -7.0   

    def __init__(self, config):
        left_pins = config.get("left_pins", [13, 19])
        right_pins = config.get("right_pins", [18, 12])
        
        self.motor_left = Motor(forward=left_pins[0], backward=left_pins[1])
        self.motor_right = Motor(forward=right_pins[0], backward=right_pins[1])
        
        self.base_speed = config.get("speed", 0.9)
        self.turn_speed = config.get("turn_speed", 0.8)
        
        self.left_trim = 1.0  
        self.right_trim = 1.0  
        
        self.speed_left = self.base_speed * self.left_trim
        self.speed_right = self.base_speed * self.right_trim

        self.vision = None
        self.arm    = None
        
        print("[CHASSIS] Motors Initialized.")

        # Shared I2C Bus
        self.i2c = busio.I2C(board.SCL, board.SDA)

        # ---------------------------------------------------------
        # BNO085 IMU SETUP
        # ---------------------------------------------------------
        try:
            # Set up the interrupt pin on GPIO5
            bno_int = DigitalInOut(board.D5)
            bno_int.direction = Direction.INPUT
            bno_int.pull = Pull.UP
            
            # Initialize without a reset pin
            self.bno = BNO08X_I2C(self.i2c, reset=None)
            
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

            print("[CHASSIS] BNO085 hardware initialized on GPIO5.\n")
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
        # INA226 BATTERY MONITOR SETUP 
        # ---------------------------------------------------------
        try:
            self.bat_alert = DigitalInOut(board.D6)
            self.bat_alert.direction = Direction.INPUT
            self.bat_alert.pull = Pull.UP

            self.ina = INA226(busnum=1, max_expected_amps=8, shunt_ohms=0.01, address=0x40)
            self.ina.configure()
            self.has_ina = True
            print("[CHASSIS] INA226 Battery Monitor Initialized on GPIO6.")
        except Exception as e:
            print(f"[CHASSIS] INA226 init failed: {e}")
            self.has_ina = False

    # ---------------------------------------------------------
    # BATTERY LOGIC
    # ---------------------------------------------------------
    def get_battery_percentage(self, voltage):
        """Piecewise curve for a 4S (12.8V) LiFePO4 Battery."""
        if voltage >= 14.0: return 100
        elif voltage >= 13.3: return 90
        elif voltage >= 13.2: return 70
        elif voltage >= 13.1: return 40
        elif voltage >= 13.0: return 30
        elif voltage >= 12.9: return 20
        elif voltage >= 12.8: return 15
        elif voltage >= 12.5: return 10
        elif voltage >= 12.0: return 5
        else: return 0

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
            print(f"[IMU WARNING] Sensor glitch: {e}")
            return None

    def get_heading_smoothed(self, samples=7):
        readings = []
        for _ in range(samples):
            h = self.get_heading()
            if h is not None:
                readings.append(h)
            time.sleep(0.010)

        if not readings:
            return None

        sin_vals = [math.sin(math.radians(r)) for r in readings]
        cos_vals = [math.cos(math.radians(r)) for r in readings]

        sin_mean = sum(sin_vals) / len(sin_vals)
        cos_mean = sum(cos_vals) / len(cos_vals)
        mean_deg = math.degrees(math.atan2(sin_mean, cos_mean)) % 360

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
            # --- EMERGENCY PROTECTIONS ---
            if self.is_tilted_dangerously():
                self.stop()
                print("\n[CHASSIS] EMERGENCY STOP: Excessive tilt detected!\n")
                return 

            if getattr(self, "has_ina", False) and not self.bat_alert.value:
                self.stop()
                print("\n[CHASSIS] EMERGENCY STOP: Battery Alert Triggered on GPIO6!\n")
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
                time_driven_so_far = time.time() - start_time
                pause_ts = time.strftime("%H:%M:%S")
                print(f"\n[SWEEP] {pause_ts} | Target detected. Pausing sweep ({time_driven_so_far:.2f}s into lane).")

                # Multi-target loop: keep picking up while the response zone has valid targets.
                # MAX_PICKUPS_PER_POSITION guards against an infinite loop if a failed grab
                # leaves the same item in the zone repeatedly.
                MAX_PICKUPS_PER_POSITION = 5
                pickup_count = 0
                while (pickup_count < MAX_PICKUPS_PER_POSITION
                       and self.vision
                       and getattr(self.vision, 'target_in_response_zone', False)):
                    pickup_count += 1
                    trash_type = getattr(self.vision, 'target_class', "Unknown")
                    conf       = getattr(self.vision, 'target_conf',  0.0)
                    print(f"[SWEEP] Pickup #{pickup_count}: {trash_type} (conf={conf:.2f})")
                    self.execute_pickup_and_return(trash_type)
                    # Let the vision thread update the zone flag before re-checking.
                    time.sleep(0.5)

                if pickup_count >= MAX_PICKUPS_PER_POSITION:
                    print("[SWEEP] Max pickups reached at this position -- continuing sweep.")

                travel_time = travel_time - time_driven_so_far
                start_time  = time.time()
                resume_ts   = time.strftime("%H:%M:%S")
                print(f"[SWEEP] {resume_ts} | Zone clear after {pickup_count} pickup(s). Resuming — {travel_time:.2f}s remaining.\n")
                
            current_heading = self.get_heading()
            
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
        t_start = time.time()
        print(f"\n[PICKUP] {time.strftime('%H:%M:%S')} | --- INITIATING AUTO-PICKUP for: {trash_type.upper()} ---")

        original_heading = self.get_heading_smoothed(samples=5)
        if original_heading is None:
            original_heading = self.get_heading() or 0.0
        print(f"[PICKUP] Saved original sweep line heading: {original_heading:.1f}°")

        if self.vision:
            self.vision.pause_tape_detection()

        # Compute grab zone boundaries — identical to auto-align tracking_loop
        zh = self.vision.zone_cfg.get("height", 90) if self.vision else 90
        oy = self.vision.zone_cfg.get("offset_y", 0) if self.vision else 0
        red_center_y = 360 - (zh / 2) + oy
        red_top    = red_center_y - (zh / 2)
        red_bottom = red_center_y + (zh / 2)
        crosshair_left  = 260
        crosshair_right = 380

        print("[PICKUP] Aligning and approaching target...")
        seq_start = time.time()
        seq_timeout = seq_start + 15.0
        grabbed = False

        while time.time() < seq_timeout:
            if not self.vision:
                break

            tx        = self.vision.target_x
            ty_top    = getattr(self.vision, 'target_y_top', None)
            ty_bottom = getattr(self.vision, 'target_y_bottom', None)

            if tx is None or ty_top is None or ty_bottom is None:
                self.stop()
                time.sleep(0.05)
                continue

            if tx < crosshair_left or tx > crosshair_right:
                center_x  = (crosshair_left + crosshair_right) / 2
                error_x   = abs(tx - center_x)
                min_turn  = 0.28
                turn_spd  = min(min_turn + (error_x / 320) * (self.turn_speed - min_turn), self.turn_speed)
                if tx < crosshair_left:
                    self.motor_left.backward(turn_spd)
                    self.motor_right.backward(turn_spd)
                else:
                    self.motor_left.forward(turn_spd)
                    self.motor_right.forward(turn_spd)
            else:
                green_height = ty_bottom - ty_top

                # Inner center box of grab zone (middle 50% of its height)
                gz_cy           = (red_top + red_bottom) / 2
                gz_inner_top    = gz_cy - (zh * 0.25)
                gz_inner_bottom = gz_cy + (zh * 0.25)

                # Inner center box of core hitbox (middle 50% of its height)
                core_cy           = (ty_top + ty_bottom) / 2
                core_inner_top    = core_cy - (green_height * 0.25)
                core_inner_bottom = core_cy + (green_height * 0.25)

                # Pickup triggers when the two center boxes overlap
                is_centered    = (core_inner_top <= gz_inner_bottom) and (core_inner_bottom >= gz_inner_top)
                is_giant_trash = (green_height >= zh) and (ty_bottom >= red_bottom)

                if is_centered or is_giant_trash:
                    self.stop()
                    grabbed = True
                    break
                else:
                    self.move_approach()

            time.sleep(0.02)

        self.stop()
        approach_time = time.time() - seq_start

        if not grabbed:
            print("[PICKUP] Timed out — trash never reached grab zone. Aborting.")
            if self.vision:
                self.vision.resume_tape_detection()
            return

        t_grab = time.time()
        print(f"[PICKUP] {time.strftime('%H:%M:%S')} | *** GRABBING TRASH ({t_grab - t_start:.2f}s since pickup start) ***")

        trash_lower = trash_type.lower()
        plastic_classes = ["general_plastic", "plastic_bottles", "plastic_bottle"]
        glass_classes   = ["glass_bottles", "glass_bottle", "glass"]

        if trash_lower in plastic_classes:
            drop_zone = 'l'
        elif trash_lower in glass_classes:
            drop_zone = 'r'
        else:
            drop_zone = 'c'

        orientation = getattr(self.vision, 'target_orientation', 'vertical') if self.vision else 'vertical'

        if self.arm:
            print(f"[PICKUP] Arm pickup ({orientation}) -> drop zone '{drop_zone}'")
            self.arm.pickup_sequence(orientation)
            self.arm.return_sequence(drop_zone)
        else:
            print("[PICKUP] No arm connected -- skipping grab.")

        t_reverse = time.time()
        print(f"[PICKUP] {time.strftime('%H:%M:%S')} | Reversing to grid position ({approach_time:.2f}s)...")
        reverse_start = time.time()
        
        approach_factor = 0.65 
        l_speed = max(0.35, self.speed_left * approach_factor)
        r_speed = max(0.35, self.speed_right * approach_factor)
        
        while time.time() - reverse_start < approach_time:
            self.motor_left.backward(l_speed)
            self.motor_right.forward(r_speed)
            time.sleep(0.02)
        self.stop()

        print("[PICKUP] Snapping back to original sweep heading...")
        self.turn_to_absolute_heading(original_heading)

        if self.vision:
            self.vision.resume_tape_detection()

        t_done = time.time()
        print(f"[PICKUP] {time.strftime('%H:%M:%S')} | --- SEQUENCE COMPLETE (total: {t_done - t_start:.2f}s) ---\n")

    def get_telemetry(self):
        yaw = self.get_heading()
        pitch = 0.0
        roll = 0.0
        
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

        telemetry = {
            "yaw": yaw_str,
            "pitch": f"{pitch}",
            "roll": f"{roll}",
            "mpu_ok": getattr(self, "has_imu", False),
            "tilt_warning": self.is_tilted_dangerously()
        }

        # Inject battery stats if INA226 is connected
        if getattr(self, "has_ina", False):
            try:
                voltage = self.ina.voltage()
                
                telemetry["battery_voltage"] = round(voltage, 2)
                telemetry["battery_percent"] = self.get_battery_percentage(voltage)
                telemetry["battery_alert"] = not self.bat_alert.value
            except Exception as e:
                print(f"[TELEMETRY WARNING] Failed to read INA226: {e}")

        return telemetry

    def turn_to_absolute_heading(self, target_heading):
        if not self.has_imu:
            print("[CHASSIS] No IMU. Cannot execute pure absolute turn.")
            return

        target_heading = target_heading % 360
        print(f"\n[CHASSIS] --- SNAPPING TO GRID HEADING: {target_heading:.2f} ---")

        DEADBAND       = 4.0
        SLOW_ZONE      = 25.0
        MIN_TURN_SPEED = 0.35
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

    def _recalibrate_heading(self, last_known=None, timeout=15.0):
        """Full mid-maneuver IMU recalibration.
        Stops motors, waits for motor magnetic field to decay, restarts the
        BNO085 calibration cycle, and blocks until the chip reports acceptable
        accuracy before returning a fresh heading reading.
        Falls back to last_known if the sensor cannot stabilise in time.
        """
        print("[IMU RECAL] Starting full recalibration -- motors stopped.")
        self.stop()
        time.sleep(2.0)  # Let motor coil magnetic field fully decay

        try:
            self.bno.begin_calibration()
            print("[IMU RECAL] Calibration cycle restarted on BNO085.")
        except Exception as e:
            print(f"[IMU RECAL] begin_calibration failed: {e}")

        # Poll until the chip confirms accuracy >= 2, or timeout
        start = time.time()
        while time.time() - start < timeout:
            try:
                accuracy = self.bno.calibration_status
            except Exception:
                accuracy = 0
            print(f"[IMU RECAL] Accuracy: {accuracy}/3")
            if accuracy >= 2:
                print(f"[IMU RECAL] Accuracy locked at {accuracy}/3 -- reading heading.")
                break
            time.sleep(0.5)
        else:
            print("[IMU RECAL] Timeout -- proceeding with best available accuracy.")

        heading = self.get_heading_smoothed(samples=15)
        if heading is not None:
            print(f"[IMU RECAL] Stable heading: {heading:.1f}°")
        else:
            print(f"[IMU RECAL] Could not get stable heading -- using last known: {last_known}")
            heading = last_known

        return heading

    def _skid_turn_to(self, target_heading, timeout=12.0):
        """Skid-steer to an absolute heading. Returns True on success, False on timeout."""
        DEADBAND          = 5.0
        SETTLE_READS      = 4
        TURN_SPEED        = self.turn_speed
        MAX_HEADING_JUMP  = 25.0  # degrees — rejects EMI-induced sensor spikes

        settled    = 0
        start_time = time.time()
        last_valid = None

        while True:
            if time.time() - start_time > timeout:
                self.stop()
                return False

            current = self.get_heading_smoothed(samples=3)
            if current is None:
                time.sleep(0.01)
                continue

            if last_valid is not None:
                jump = abs((current - last_valid + 540) % 360 - 180)
                if jump > MAX_HEADING_JUMP:
                    print(f"[IMU GLITCH] {last_valid:.1f}° -> {current:.1f}° ({jump:.1f}° jump) -- full recalibration")
                    recal = self._recalibrate_heading(last_known=last_valid)
                    if recal is not None:
                        last_valid = recal
                    continue
            last_valid = current

            error = (target_heading - current + 540) % 360 - 180

            if abs(error) < DEADBAND:
                self.stop()
                time.sleep(0.05)
                settled += 1
                if settled >= SETTLE_READS:
                    return True
                time.sleep(0.02)
                continue
            else:
                settled = 0

            if error > 0:
                self.motor_left.backward(TURN_SPEED)
                self.motor_right.backward(TURN_SPEED)
            else:
                self.motor_left.forward(TURN_SPEED)
                self.motor_right.forward(TURN_SPEED)

            time.sleep(0.01)

    def arc_turn_to_heading(self, target_heading, turn_deg):
        target_heading = target_heading % 360
        FORWARD_TIME   = 2.5

        direction = "RIGHT" if turn_deg < 0 else "LEFT"
        print(f"[CHASSIS] Skid U-turn -> {target_heading:.1f}° ({direction})")

        current = self.get_heading_smoothed(samples=5)
        if current is None:
            current = self.get_heading() or 0.0

        mid_offset  = -90.0 if turn_deg < 0 else +90.0
        mid_heading = (current + mid_offset) % 360

        time.sleep(0.5)
        # Phase 1: first 90° skid turn
        print(f"[CHASSIS] Phase 1: Skid turn to {mid_heading:.1f}°")
        if not self._skid_turn_to(mid_heading):
            print("[CHASSIS] Phase 1 timeout -- aborting U-turn.")
            return None

        # Phase 2: drive forward to form the "_" base of the U
        print(f"[CHASSIS] Phase 2: Forward drive {FORWARD_TIME:.1f}s")
        drive_start = time.time()
        while time.time() - drive_start < FORWARD_TIME:
            self.motor_left.forward(self.speed_left)
            self.motor_right.backward(self.speed_right)
            time.sleep(0.02)
        self.stop()
        # Extended pause: lets residual motor magnetic field decay before the IMU is trusted.
        time.sleep(3.0)

        # Recalibrate IMU before Phase 3 so error is computed from a fresh, settled reading
        print("[CHASSIS] Recalibrating IMU heading before Phase 3...")
        recal = self.get_heading_smoothed(samples=10)
        if recal is not None:
            print(f"[CHASSIS] IMU settled at {recal:.1f}° | target: {target_heading:.1f}°")
        else:
            print("[CHASSIS] IMU recal returned None -- proceeding anyway.")

        # Phase 3: second 90° skid turn to reach target heading
        print(f"[CHASSIS] Phase 3: Skid turn to {target_heading:.1f}°")
        if not self._skid_turn_to(target_heading):
            print("[CHASSIS] Phase 3 timeout -- aborting U-turn.")
            return None

        final = self.get_heading_smoothed()
        if final is not None:
            print(f"[CHASSIS] U-turn complete. Final heading: {final:.1f}°")
        return final

    def sweep_area(self, grid_size_cm=None):
        rest_time = 0.5

        if grid_size_cm:
            lane_width_cm = 50.0
            lanes     = max(1, int(grid_size_cm / lane_width_cm))
            lane_time = grid_size_cm * (6.1 / 170.0)
            print(f"\n[SWEEP] Starting sweep: {lanes} lanes")
        else:
            lanes     = 999
            lane_time = 600.0  # red tape boundary will always cut this short
            print(f"\n[SWEEP] Boundary mode: driving until red tape, U-turning indefinitely.")

        current_heading = self.get_heading_smoothed(samples=10)
        if current_heading is None:
            current_heading = self.get_heading() or 0.0

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
            
            if self.vision:
                self.vision.pause_tape_detection()
                
            self.arc_turn_to_heading((current_heading + turn_deg) % 360, turn_deg)

            self.stop()
            time.sleep(1.0)
            
            if self.vision:
                self.vision.resume_tape_detection()
                
            new_heading = self.get_heading_smoothed(samples=10)
            if new_heading is not None:
                current_heading = new_heading
                
            print(f"[SWEEP] Fresh IMU heading after arc: {current_heading:.1f}° — this is the new straight")

            time.sleep(0.3)
            use_right_turn = not use_right_turn

        print("[SWEEP] --- SWEEP COMPLETE ---\n")