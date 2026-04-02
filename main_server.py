#!/home/pifivedbz1/Desktop/tidey_workspace/venv/bin/python3

import os
import time
import threading
import json
from flask import Flask, render_template, Response, send_from_directory, jsonify, request

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

@app.route('/api/arm/angles', methods=['GET'])
def get_arm_angles():
    # Map the list of current positions to their readable names
    angles = {
        "base": robot_arm.current_pos[0],
        "shoulder": robot_arm.current_pos[1],
        "elbow": robot_arm.current_pos[2],
        "wpitch": robot_arm.current_pos[3],
        "wroll": robot_arm.current_pos[4],
        "gripper": robot_arm.current_pos[5]
    }
    return jsonify(angles)

@app.route('/cmd/arm/move', methods=['POST'])
def manual_arm_move():
    # Receive the joint name and angle from the web UI
    data = request.json
    joint = data.get('joint')
    angle = data.get('angle')
    
    if joint and angle is not None:
        # Run in a background thread so the camera stream doesn't freeze!
        threading.Thread(target=robot_arm.smooth_move, args=(joint, int(angle))).start()
        return jsonify({"status": "moving", "joint": joint, "angle": angle})
        
    return jsonify({"status": "error", "message": "Invalid data"}), 400

@app.route('/cmd/chassis/sweep', methods=['POST'])
def control_chassis_sweep():
    data = request.json
    grid_size = float(data.get('distance', 0))
    
    if grid_size > 0:
        # Run the long sweep sequence in a background thread
        threading.Thread(target=robot_base.sweep_area, args=(grid_size,)).start()
        return jsonify({"status": "sweeping", "grid_size": grid_size})
        
    return jsonify({"status": "error", "message": "Invalid grid size"}), 400

@app.route('/cmd/chassis/distance', methods=['POST'])
def control_chassis_distance():
    data = request.json
    distance = float(data.get('distance', 0))
    direction = data.get('direction', 'w') # 'w' for forward, 's' for backward
    
    if distance > 0:
        # Run in a background thread to prevent the video stream from freezing
        threading.Thread(target=robot_base.move_set_distance, args=(distance, direction)).start()
        return jsonify({"status": "moving", "distance": distance, "direction": direction})
        
    return jsonify({"status": "error", "message": "Invalid distance"}), 400

if __name__ == '__main__':
    try:
        print("\n[SYSTEM] Server starting on port 5000...")
        app.run(host='0.0.0.0', port=5000, debug=False) 
    except KeyboardInterrupt:
        print("\n[SYSTEM] Shutting down...")
        eyes.stop()
        # robot_base.stop()
