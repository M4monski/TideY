#!/home/pifivedbz1/Desktop/tidey_workspace/venv/bin/python3

import os
import time
import threading
import json
from flask import Flask, render_template, Response, send_from_directory, jsonify

# Import your clean hardware and AI classes
from chassis import Chassis
from arm import RoboticArm
from vision import VisionSystem

app = Flask(__name__)
IMAGE_DIR = "images"
os.makedirs(IMAGE_DIR, exist_ok=True)

# 1. Load configuration
try:
    with open('config.json') as f:
        config = json.load(f)
except FileNotFoundError:
    print("ERROR: config.json not found! Using default empty dictionaries.")
    config = {}

# 2. Initialize Subsystems
robot_base = Chassis(config.get('chassis', {}))
robot_arm = RoboticArm(config.get('arm', {}))

eyes = VisionSystem(config.get('vision', {}))
eyes.start_stream()

# 3. Web Routes
@app.route('/')
def index():
    return render_template('index.html')

def video_feed_generator():
    while eyes.stream_active:
        frame = eyes.get_frame()
        if frame:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(video_feed_generator(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/capture', methods=['POST'])
def capture():
    filename = f"photo_{int(time.time())}.jpg"
    filepath = os.path.join(IMAGE_DIR, filename)
    
    # Just call the clean method from vision.py
    eyes.capture_high_res(filepath)
    
    return jsonify({"success": True, "filename": filename})

@app.route('/images/<filename>')
def serve_image(filename):
    return send_from_directory(IMAGE_DIR, filename)

@app.route('/cmd/chassis/<direction>', methods=['POST'])
def control_chassis(direction):
    if direction == 'w': robot_base.move_forward()
    elif direction == 's': robot_base.move_backward()
    elif direction == 'a': robot_base.spin_left()
    elif direction == 'd': robot_base.spin_right()
    elif direction == 'x': robot_base.stop()
    return jsonify({"status": "ok", "action": direction})   
@app.route('/cmd/arm/<action>', methods=['POST'])
def control_arm(action):
    # We run the arm in a background thread so the camera stream doesn't freeze!
    if action == 'pickup': 
        threading.Thread(target=robot_arm.pickup_sequence).start()
    elif action == 'home': 
        threading.Thread(target=robot_arm.home_sequence).start()
    elif action == 'drop_l': 
        threading.Thread(target=robot_arm.return_sequence, args=('l',)).start()
    elif action == 'drop_c': 
        threading.Thread(target=robot_arm.return_sequence, args=('c',)).start()
    elif action == 'drop_r': 
        threading.Thread(target=robot_arm.return_sequence, args=('r',)).start()
    return jsonify({"status": "ok", "action": action})

if __name__ == '__main__':
    try:
        print("\n[SYSTEM] Server starting on port 5000...")
        app.run(host='0.0.0.0', port=5000, debug=False) 
    except KeyboardInterrupt:
        print("\n[SYSTEM] Shutting down...")
        eyes.stop()
        # robot_base.stop()