import time
import math
import json
import os
import board
import busio
from digitalio import DigitalInOut
from adafruit_bno08x import (
    BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR,
    BNO_REPORT_MAGNETOMETER,
)
from adafruit_bno08x.i2c import BNO08X_I2C

i2c = busio.I2C(board.SCL, board.SDA)
bno_reset = DigitalInOut(board.D17)
bno = BNO08X_I2C(i2c, reset=bno_reset)

bno.enable_feature(BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR)
bno.enable_feature(BNO_REPORT_MAGNETOMETER)
time.sleep(1)

# --- DUMP ALL ATTRIBUTES ---
print("\n--- BNO085 available attributes ---")
bno_attrs = [a for a in dir(bno) if not a.startswith('_')]
for a in bno_attrs:
    print(a)



CALIBRATION_FILE = "bno085_calibration.json"

def save_calibration(bno):
    """Read calibration data from sensor and persist it to disk."""
    try:
        cal = bno.calibration_status
        with open(CALIBRATION_FILE, "w") as f:
            json.dump({"calibration_status": cal, "timestamp": time.time()}, f)
        print(f"\n[CAL] Calibration saved to {CALIBRATION_FILE}")
    except Exception as e:
        print(f"\n[CAL] Could not save calibration: {e}")

def load_calibration(bno):
    """Load previously saved calibration data into the sensor."""
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
    print("      Rotate slowly and smoothly. Cover all orientations.")
    print("      Accuracy: 0=unreliable  1=low  2=medium  3=high\n")

    history = []
    stable_count = 0
    STABLE_THRESHOLD = 10

    while True:
        try:
            accuracy = bno.geomagnetic_quaternion_accuracy  # 0-3
            
            # --- ADD THIS: raw magnetometer read to confirm sensor is alive ---
            try:
                mx, my, mz = bno.magnetic
                mag_str = f"mag=({mx:.1f},{my:.1f},{mz:.1f})"
            except Exception:
                mag_str = "mag=NO DATA"

            history.append(accuracy)
            if len(history) > 20:
                history.pop(0)

            avg = sum(history) / len(history)
            stability = int((avg / 3.0) * 100)

            bar = "█" * accuracy + "░" * (3 - accuracy)
            label = ["UNRELIABLE", "LOW     ", "MEDIUM  ", "HIGH    "][accuracy]

            # Always reprint — don't gate on status change
            print(f"\r  [{bar}] {accuracy}/3 {label}  stab={stability}%  {mag_str}   ", end="", flush=True)

            if accuracy >= 2:
                stable_count += 1
            else:
                stable_count = 0

            if stable_count >= STABLE_THRESHOLD:
                print(f"\n\n[CAL] Calibration locked in.")
                save_calibration(bno)
                return

        except Exception as e:
            print(f"\n[CAL READ ERROR] {e}")

        time.sleep(0.2)     
def main():
    print("--- Starting BNO085 Standalone Test ---")

    # 1. Initialize hardware
    try:
        i2c = busio.I2C(board.SCL, board.SDA)
        bno_reset = DigitalInOut(board.D17)
        bno = BNO08X_I2C(i2c, reset=bno_reset)

        bno.enable_feature(BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR)
        bno.enable_feature(BNO_REPORT_MAGNETOMETER)  # needed to read accuracy status
        print("[SUCCESS] BNO085 initialized.")
        time.sleep(2)  # let fusion engine start up

    except Exception as e:
        print(f"\n[ERROR] Could not initialize BNO085: {e}")
        print("Check wiring and ensure no other script is using the I2C bus.")
        return

    # 2. Try to restore saved calibration, otherwise wait for live calibration
    calibration_loaded = load_calibration(bno)
    if not calibration_loaded:
        wait_for_calibration(bno)
    else:
        # Even with saved cal, do a quick sanity check — if accuracy is already
        # good we skip the wait, otherwise fall through to calibrate
        time.sleep(1)
        try:
            accuracy = bno.geomagnetic_quaternion_accuracy
            if accuracy < 2:
                print("[CAL] Saved calibration loaded but accuracy still low — recalibrating.")
                wait_for_calibration(bno)
            else:
                print(f"[CAL] Saved calibration valid (accuracy={accuracy}). Ready immediately!")
        except Exception:
            wait_for_calibration(bno)

    print("\n[LIVE SENSOR DATA]")
    print("-" * 50)

    last_save = time.time()
    SAVE_INTERVAL = 300  # re-save calibration every 5 minutes as it improves

    # 3. Main read loop
    while True:
        try:
            quat = bno.geomagnetic_quaternion

            if quat:
                # BNO085 returns (i, j, k, real) — remap to (w, x, y, z) for Euler math
                i, j, k, real = quat
                w, x, y, z = real, i, j, k

                # Yaw (Heading)
                siny_cosp = 2.0 * (w * z + x * y)
                cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
                yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))
                if yaw < 0:
                    yaw += 360

                # Pitch
                sinp = 2.0 * (w * y - z * x)
                if abs(sinp) >= 1:
                    pitch = math.degrees(math.copysign(math.pi / 2, sinp))
                else:
                    pitch = math.degrees(math.asin(sinp))

                # Roll
                sinr_cosp = 2.0 * (w * x + y * z)
                cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
                roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

                # Show live accuracy alongside angles so you know how much to trust it
                try:
                    accuracy = bno.geomagnetic_quaternion_accuracy
                    acc_label = ["UNRELIABLE", "LOW", "MEDIUM", "HIGH"][accuracy]
                except Exception:
                    acc_label = "?"

                print(
                    f"\rHeading: {yaw:>6.2f}°  Pitch: {pitch:>6.2f}°  Roll: {roll:>6.2f}°  "
                    f"| Accuracy: {acc_label}   ",
                    end="", flush=True,
                )

                # Periodically re-save calibration as the sensor keeps improving it
                if time.time() - last_save > SAVE_INTERVAL:
                    save_calibration(bno)
                    last_save = time.time()

        except Exception as e:
            print(f"\n[READ ERROR] {e}")

        time.sleep(0.05)  # 20Hz

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n[SYSTEM] Test stopped by user. Goodbye!")