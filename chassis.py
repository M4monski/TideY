from gpiozero import Motor

class Chassis:
    def __init__(self, config):
        """
        Initializes the motor controller using settings from config.json
        """
        left_pins = config.get("left_pins", [13, 19])
        right_pins = config.get("right_pins", [18, 12])
        
        self.motor_left = Motor(forward=left_pins[0], backward=left_pins[1])
        self.motor_right = Motor(forward=right_pins[0], backward=right_pins[1])
        
        self.speed = config.get("speed", 0.7)
        self.turn_speed = config.get("turn_speed", 0.6)
        
        print("[CHASSIS] Motors Initialized.")

    def move_forward(self):
        print("[CHASSIS] Moving Forward")
        self.motor_left.forward(self.speed)
        self.motor_right.backward(self.speed)

    def move_backward(self):
        print("[CHASSIS] Moving Backward")
        self.motor_left.backward(self.speed)
        self.motor_right.forward(self.speed)

    def spin_left(self):
        print("[CHASSIS] Spinning Left")
        self.motor_left.backward(self.turn_speed)
        self.motor_right.backward(self.turn_speed)

    def spin_right(self):
        print("[CHASSIS] Spinning Right")
        self.motor_left.forward(self.turn_speed)
        self.motor_right.forward(self.turn_speed)

    def stop(self):
        print("[CHASSIS] Stopped")
        self.motor_left.stop()
        self.motor_right.stop()