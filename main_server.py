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



@app.route('/cmd/vision/response_zone', methods=['POST'])
def update_response_zone():
    data = request.json
    
    # 1. Instantly update the live camera feed in memory
    if 'bottom_width' in data: 
        eyes.response_cfg['bottom_width'] = int(data['bottom_width'])
    if 'top_width' in data: 
        eyes.response_cfg['top_width'] = int(data['top_width'])
    if 'height' in data: 
        eyes.response_cfg['height'] = int(data['height'])
    if 'offset_y' in data: 
        eyes.response_cfg['offset_y'] = int(data['offset_y'])
        
    # 2. Permanently save the new settings to config.json
    try:
        with open('config.json', 'r') as f:
            full_config = json.load(f)
            
        if 'vision' not in full_config:
            full_config['vision'] = {}
            
        full_config['vision']['response_zone'] = eyes.response_cfg
        
        with open('config.json', 'w') as f:
            json.dump(full_config, f, indent=2)
            
        print("[SERVER] Response Zone permanently saved to config.json!")
    except Exception as e:
        print(f"[SERVER] Error saving to config: {e}")
        
    return jsonify({"status": "updated", "config": eyes.response_cfg})

# --- AUTONOMOUS TRACKING LOOP ---
tracking_active = False

def tracking_loop():
    global tracking_active
    print("[AUTO] Core Hitbox Target Lock loop started.")
    while tracking_active:
        try:
            tx = eyes.target_x
            
            # Use getattr to prevent the server from crashing if files are mismatched
            ty_top = getattr(eyes, 'target_y_top', None)
            ty_bottom = getattr(eyes, 'target_y_bottom', None)
            
            # Safety Check!
            if ty_top is None and hasattr(eyes, 'target_y'):
                print("[WARNING] Tracking stopped! Your vision.py is using the old code. Please update vision.py to the Core Hitbox version.")
                tracking_active = False
                break
            
            if tx is not None and ty_top is not None and ty_bottom is not None:
                
                # --- CROSSHAIR MATH ---
                zh = eyes.zone_cfg.get("height", 90)
                oy = eyes.zone_cfg.get("offset_y", 0)
                crosshair_y = 360 - (zh / 2) + oy
                
                # WIDENED CROSSHAIR: Gives the robot a 120-pixel wide forgiving window!
                crosshair_left = 260 
                crosshair_right = 380
                # ----------------------

                # ALIGNMENT: 
                if tx < crosshair_left:
                    robot_base.spin_left()
                elif tx > crosshair_right:
                    robot_base.spin_right()
                else:
                    # APPROACH: Target is in the wide vertical lane!
                    if crosshair_y > ty_bottom:
                        robot_base.move_approach()
                    else:
                        # BULLSEYE! 
                        robot_base.stop()
                        print("[AUTO] Crosshair hit the Core Hitbox! Initiating Pickup.")
                        
                        tracking_active = False 
                        threading.Thread(target=robot_arm.pickup_sequence).start()
            else:
                robot_base.stop() 
                
        except Exception as e:
            print(f"[AUTO ERROR] {e}")
            robot_base.stop()
            
        time.sleep(0.05) 
        
    robot_base.stop() 
    print("[AUTO] Tracking loop stopped.")

@app.route('/cmd/chassis/track/<state>', methods=['POST'])
def set_tracking(state):
    global tracking_active
    if state == 'on' and not tracking_active:
        tracking_active = True
        threading.Thread(target=tracking_loop, daemon=True).start()
    elif state == 'off':
        tracking_active = False
    return jsonify({"status": "ok", "tracking": tracking_active})
# --------------------------------

# --- AUTONOMOUS TRACKING LOOP ---
tracking_active = False

def tracking_loop():
    global tracking_active
    print("[AUTO] Full-Containment Target Lock loop started.")
    while tracking_active:
        try:
            tx = eyes.target_x
            ty_top = getattr(eyes, 'target_y_top', None)
            ty_bottom = getattr(eyes, 'target_y_bottom', None)
            
            # Safety Check
            if ty_top is None and hasattr(eyes, 'target_y'):
                print("[WARNING] Update vision.py to the Core Hitbox version.")
                tracking_active = False
                break
            
            if tx is not None and ty_top is not None and ty_bottom is not None:
                
                # --- RED BOX (GRAB ZONE) BOUNDARIES ---
                zh = eyes.zone_cfg.get("height", 90)
                oy = eyes.zone_cfg.get("offset_y", 0)
                
                # Calculate exactly where the top and bottom of the Red Box are on the screen
                red_center_y = 360 - (zh / 2) + oy
                red_top = red_center_y - (zh / 2)
                red_bottom = red_center_y + (zh / 2)
                
                # WIDENED CROSSHAIR: (120-pixel wide vertical lane)
                crosshair_left = 260 
                crosshair_right = 380
                # --------------------------------------

                # ALIGNMENT: Keep the center of the trash in the lane
                if tx < crosshair_left:
                    robot_base.spin_left()
                elif tx > crosshair_right:
                    robot_base.spin_right()
                else:
                    # APPROACH STAGE: The trash is in front of us!
                    
                    green_height = ty_bottom - ty_top
                    
                    # Check 1: FULL CONTAINMENT
                    # Is the Green Box fully inside the Red Box?
                    is_contained = (ty_top >= red_top) and (ty_bottom <= red_bottom)
                    
                    # Check 2: SAFETY OVERRIDE FOR GIANT TRASH
                    # If the trash is taller than the red box, it can never fully fit.
                    # Instead, we trigger when its bottom edge hits the bottom of the red box.
                    is_giant_trash = (green_height >= zh) and (ty_bottom >= red_bottom)
                    
                    # If EITHER condition is true, slam the brakes!
                    if is_contained or is_giant_trash:
                        robot_base.stop()
                        print("[AUTO] Green Box fully covered by Grab Zone! Initiating Pickup.")
                        
                        tracking_active = False 
                        threading.Thread(target=robot_arm.pickup_sequence).start()
                    else:
                        # The Green Box hasn't fully entered the Red Box yet. Keep creeping forward.
                        robot_base.move_approach()
            else:
                robot_base.stop() 
                
        except Exception as e:
            print(f"[AUTO ERROR] {e}")
            robot_base.stop()
            
        time.sleep(0.05) 
        
    robot_base.stop() 
    print("[AUTO] Tracking loop stopped.")

if __name__ == '__main__':
    try:
        print("\n[SYSTEM] Server starting on port 5000...")
        app.run(host='0.0.0.0', port=5000, debug=False) 
    except KeyboardInterrupt:
        print("\n[SYSTEM] Shutting down...")
        eyes.stop()
        # robot_base.stop()