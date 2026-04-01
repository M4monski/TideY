import time
import json
import os
import board
import busio
from adafruit_pca9685 import PCA9685

class RoboticArm:
    def __init__(self, config):
        """
        Initializes the 6-DOF arm, PCA9685, and restores last known positions.
        """
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(self.i2c)
        self.pca.frequency = 50

        # Pin Assignments (PCA9685 0-indexed)
        self.pins = {
            "base": 0,
            "shoulder": 2,
            "elbow": 4,
            "wpitch": 6,
            "wroll": 8,
            "gripper": 10
        }

        # PWM Limits & Homing
        self.tick_min = 150
        self.tick_max = 600
        self.home_pos = config.get("home_pos", [20, 60, 180, 10, 237, 115])
        self.pause_time = config.get("pause_between_joints", 0.5)
        
        # State Memory
        self.current_pos = [0] * 6
        self.eeprom_file = "eeprom_state.json"
        
        self._load_eeprom()
        self._startup_sequence()

    def _load_eeprom(self):
        if os.path.exists(self.eeprom_file):
            try:
                with open(self.eeprom_file, 'r') as f:
                    self.current_pos = json.load(f)
            except Exception:
                self.current_pos = list(self.home_pos)
        else:
            self.current_pos = list(self.home_pos)
            
        # Sanity check values
        for i in range(6):
            if self.current_pos[i] < 0 or self.current_pos[i] > 270:
                self.current_pos[i] = self.home_pos[i]

    def _save_eeprom(self):
        with open(self.eeprom_file, 'w') as f:
            json.dump(self.current_pos, f)

    def _map_range(self, x, in_min, in_max, out_min, out_max):
        return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

    def _move_pca(self, channel, angle, max_deg=270):
        angle = max(0, min(angle, max_deg))
        ticks = self._map_range(angle, 0, max_deg, self.tick_min, self.tick_max)
        duty_cycle = int((ticks / 4096.0) * 65535.0)
        self.pca.channels[channel].duty_cycle = duty_cycle

    def smooth_move(self, joint_name, target_angle, max_deg=270):
        channel = self.pins[joint_name]
        
        # Map joint name to index for current_pos tracking
        joint_indices = ["base", "shoulder", "elbow", "wpitch", "wroll", "gripper"]
        index = joint_indices.index(joint_name)
        
        start_angle = self.current_pos[index]
        if start_angle == target_angle:
            return

        step = 1 if target_angle > start_angle else -1

        for i in range(start_angle, target_angle, step):
            self._move_pca(channel, i, max_deg)
            time.sleep(0.02)
        
        self._move_pca(channel, target_angle, max_deg)
        self.current_pos[index] = target_angle
        self._save_eeprom()

    def _startup_sequence(self):
        print("[ARM] Restoring last known positions...")
        joint_names = ["base", "shoulder", "elbow", "wpitch", "wroll", "gripper"]
        for i, name in enumerate(joint_names):
            self._move_pca(self.pins[name], self.current_pos[i])
        
        time.sleep(0.5)
        self.home_sequence()
        print("[ARM] Ready.")

    # --- AUTOMATION ALGORITHMS ---
    def pickup_sequence(self):
        print("[ARM] Executing Pickup Sequence...")
        self.smooth_move("base", 20)
        time.sleep(self.pause_time)
        self.smooth_move("shoulder", 100)
        time.sleep(self.pause_time)
        self.smooth_move("elbow", 150)
        time.sleep(self.pause_time)
        self.smooth_move("wpitch", 90)
        time.sleep(self.pause_time)
        self.smooth_move("wroll", 237)
        time.sleep(self.pause_time)
        self.smooth_move("gripper", 190)
        time.sleep(self.pause_time)

    def return_sequence(self, drop_zone='c'):
        print(f"[ARM] Executing Return Sequence (Zone: {drop_zone})...")
        base_target = 20
        if drop_zone == 'l': base_target = 30
        elif drop_zone == 'r': base_target = 0

        self.smooth_move("base", base_target)
        time.sleep(self.pause_time)
        self.smooth_move("shoulder", 30)
        time.sleep(self.pause_time)
        self.smooth_move("elbow", 240)
        time.sleep(self.pause_time)
        self.smooth_move("wpitch", 0)
        time.sleep(self.pause_time)
        self.smooth_move("wroll", 237)
        time.sleep(self.pause_time)
        self.smooth_move("gripper", 115)
        time.sleep(self.pause_time)

    def home_sequence(self):
        print("[ARM] Returning to Home...")
        joint_names = ["base", "shoulder", "elbow", "wpitch", "wroll", "gripper"]
        for i, name in enumerate(joint_names):
            self.smooth_move(name, self.home_pos[i])
            time.sleep(self.pause_time)