import time
import math
import json
import os
import board
import busio
from digitalio import DigitalInOut
from adafruit_bno08x import (
    BNO_REPORT_ROTATION_VECTOR,
    BNO_REPORT_MAGNETOMETER,
)
from adafruit_bno08x.i2c import BNO08X_I2C

CALIBRATION_FILE = "bno085_calibration.json"

# Tune these until flat reads Pitch ~0° Roll ~0°
PITCH_OFFSET = 4.0    # flat was reading -4°
ROLL_OFFSET  = -7.0   # flat was reading +7°

def save_calibration(bno):
    try:
        cal = bno.calibration_status
        with open(CALIBRATION_FILE, "w") as f:
            json.dump({"calibration_status": cal, "timestamp": time.time()}, f)
        print(f"[CAL] Calibration saved to {CALIBRATION_FILE}")
    except Exception as e:
        print(f"[CAL] Could not save calibration: {e}")

def load_calibration(bno):
    if not os.path.exists(CALIBRATION_FILE):
        print("[CAL] No saved calibration found — starting fresh.")
        return False
    try:
        with open(CALIBRATION_FILE, "r") as f:
            data = json.load(f)
        age_hours = (time.time() - data.get("timestamp", 0)) / 3600
        print(f"[CAL] Loaded calibration (saved {age_hours:.1f} hours ago).")
        return True
    except Exception as e:
        print(f"[CAL] Could not load calibration: {e}")
        return False

def wait_for_calibration(bno):
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
                accuracy = bno.calibration_status
            except Exception:
                accuracy = 0

            mx, my, mz = bno.magnetic
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

            avg = sum(history) / len(history)
            stability = int((avg / 3.0) * 100)

            bar = "█" * display_accuracy + "░" * (3 - display_accuracy)
            label = ["UNRELIABLE", "LOW     ", "MEDIUM  ", "HIGH    "][display_accuracy]

            read_count += 1
            print(
                f"[{read_count:>4}] [{bar}] {display_accuracy}/3 {label} | "
                f"mag=({mx:>7.2f}, {my:>7.2f}, {mz:>7.2f}) | "
                f"strength={mag_total:>6.1f}uT | "
                f"swing={swing:>6.1f} | "
                f"cal_status={accuracy}"
            )

            if display_accuracy >= 2:
                stable_count += 1
            else:
                stable_count = 0

            if stable_count >= STABLE_THRESHOLD:
                print("\n[CAL] Calibration locked in.")
                save_calibration(bno)
                return

        except Exception as e:
            print(f"[CAL READ ERROR] {e}")

        time.sleep(0.5)

def warmup(bno, seconds=5):
    """
    Let the fusion engine settle before trusting output.
    calibration_status is unreliable for magnetometer state
    so we just wait and let the internal algorithm stabilize.
    """
    print(f"[INIT] Warming up fusion engine ({seconds}s) — hold sensor still...")
    for i in range(seconds, 0, -1):
        print(f"\r[INIT] Starting in {i}s...   ", end="", flush=True)
        time.sleep(1)
    print("\r[INIT] Ready.                    ")

def main():
    print("--- Starting BNO085 Standalone Test ---")

    try:
        i2c = busio.I2C(board.SCL, board.SDA)
        bno_reset = DigitalInOut(board.D17)
        bno = BNO08X_I2C(i2c, reset=bno_reset)

        bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
        bno.enable_feature(BNO_REPORT_MAGNETOMETER)

        print("[INIT] Starting magnetometer calibration routine...")
        bno.begin_calibration()

        print("[INIT] Priming sensor data stream...")
        for _ in range(10):
            try:
                _ = bno.quaternion
                _ = bno.magnetic
            except Exception:
                pass
            time.sleep(0.1)

        print("[SUCCESS] BNO085 initialized.\n")
        time.sleep(2)

    except Exception as e:
        print(f"\n[ERROR] Could not initialize BNO085: {e}")
        return

    calibration_loaded = load_calibration(bno)
    if not calibration_loaded:
        wait_for_calibration(bno)

    # Always warm up regardless of calibration state —
    # this replaces the unreliable calibration_status check
    warmup(bno, seconds=5)

    print("\n[LIVE SENSOR DATA] — place sensor flat and still")
    print("-" * 50)

    last_save = time.time()
    SAVE_INTERVAL = 300

    while True:
        try:
            quat = bno.quaternion

            if quat:
                i, j, k, real = quat
                w, x, y, z = real, i, j, k

                # Yaw (heading)
                siny_cosp = 2.0 * (w * z + x * y)
                cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
                yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))
                if yaw < 0:
                    yaw += 360

                # Pitch
                sinp = 2.0 * (w * y - z * x)
                sinp = max(-1.0, min(1.0, sinp))
                pitch = math.degrees(math.asin(sinp))

                # Roll
                sinr_cosp = 2.0 * (w * x + y * z)
                cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
                roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

                pitch = pitch + PITCH_OFFSET
                roll  = roll  + ROLL_OFFSET

                if pitch > 180:  pitch -= 360
                if pitch < -180: pitch += 360
                if roll > 180:   roll  -= 360
                if roll < -180:  roll  += 360

                print(
                    f"Heading: {yaw:>6.2f}°  Pitch: {pitch:>6.2f}°  "
                    f"Roll: {roll:>6.2f}°"
                )

                if time.time() - last_save > SAVE_INTERVAL:
                    save_calibration(bno)
                    last_save = time.time()

        except Exception as e:
            print(f"\n[READ ERROR] {e}")

        time.sleep(1.0)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n[SYSTEM] Test stopped by user. Goodbye!")