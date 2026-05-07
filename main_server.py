#!/home/pifivedbz1/Desktop/tidey_workspace/venv/bin/python3

import os
import time
import threading
import json
import requests as http_requests
from datetime import datetime, timezone, timedelta
from flask import jsonify
from flask import Flask, render_template, Response, send_from_directory, jsonify, request

# Import your clean hardware and AI classes
from chassis import Chassis
from arm import RoboticArm
from vision import VisionSystem

# --- GLOBAL STATUS ---
robot_status = "Idle"
# ---------------------

# --- BIN VOLUME TRACKING ---
BIN_VOLUME_FILE = "bin_volumes.json"
BIN_VOLUME_INCREMENTS = {
    "general_plastic":  1,   # +1% per pickup
    "plastic_bottles":  7,   # +7% per pickup
    "glass_bottles":    8,   # +8% per pickup
}

def _load_bin_volumes():
    if os.path.exists(BIN_VOLUME_FILE):
        try:
            with open(BIN_VOLUME_FILE, 'r') as f:
                return json.load(f)
        except Exception:
            pass
    return {"plastic": 0.0, "glass": 0.0}

def _save_bin_volumes(volumes):
    with open(BIN_VOLUME_FILE, 'w') as f:
        json.dump(volumes, f, indent=2)

bin_volumes = _load_bin_volumes()

def record_bin_pickup(trash_class):
    """Adds the appropriate volume percentage to the correct bin after a successful pickup."""
    safe = trash_class.lower()
    increment = BIN_VOLUME_INCREMENTS.get(safe, 0)
    if increment == 0:
        return
    if safe == "glass_bottles":
        bin_volumes["glass"] = min(100.0, bin_volumes["glass"] + increment)
        _check_bin_alert("glass", bin_volumes["glass"])
    else:
        bin_volumes["plastic"] = min(100.0, bin_volumes["plastic"] + increment)
        _check_bin_alert("plastic", bin_volumes["plastic"])
    _save_bin_volumes(bin_volumes)
    print(f"[BIN] {trash_class} -> +{increment}% | Plastic: {bin_volumes['plastic']:.1f}% | Glass: {bin_volumes['glass']:.1f}%")
# ---------------------------

# --- SMS ALERT SYSTEM (PhilSMS) ---
SMS_ALERT_THRESHOLD  = 90.0   # bin % that triggers a text
SMS_COOLDOWN_SECONDS = 60     # max 1 message per minute per alert type

_sms_last_sent = {"plastic": 0.0, "glass": 0.0}  # epoch timestamps

def _to_ph_intl(phone: str) -> str:
    """Converts 09xxxxxxxxx → 639xxxxxxxxx as required by PhilSMS."""
    phone = phone.strip()
    if phone.startswith("0"):
        return "63" + phone[1:]
    if phone.startswith("+"):
        return phone[1:]
    return phone

def _send_sms(message: str, sms_cfg: dict) -> bool:
    """Sends an SMS via PhilSMS (dashboard.philsms.com)."""
    token = sms_cfg.get("philsms_token", "")
    phone = sms_cfg.get("phone", "")

    if not token or not phone:
        print("[NOTIFY] PhilSMS token or phone not configured — skipping.")
        return False

    recipient = _to_ph_intl(phone)

    try:
        resp = http_requests.post(
            "https://dashboard.philsms.com/api/v3/sms/send",
            headers={
                "Authorization": f"Bearer {token}",
                "Content-Type":  "application/json",
                "Accept":        "application/json",
            },
            json={
                "recipient": recipient,
                "sender_id": "PhilSMS",
                "type":      "plain",
                "message":   message,
            },
            timeout=15,
        )
        data = resp.json()
        if resp.status_code == 200 and data.get("status") == "success":
            print(f"[NOTIFY] PhilSMS sent OK to {recipient}: {message[:60]}...")
            return True
        else:
            print(f"[NOTIFY] PhilSMS failed ({resp.status_code}): {data}")
            return False
    except Exception as e:
        print(f"[NOTIFY] Request error: {e}")
        return False

def _check_bin_alert(bin_name: str, current_pct: float):
    """Fires an SMS alert if the bin crossed 90% and the cooldown has passed."""
    global _sms_last_sent

    if current_pct < SMS_ALERT_THRESHOLD:
        return

    now = time.time()
    if (now - _sms_last_sent[bin_name]) < SMS_COOLDOWN_SECONDS:
        return  # still within the 5-minute cooldown

    bin_label    = "Plastic Bin (Left)"  if bin_name == "plastic" else "Glass Bin (Right)"
    other_name   = "glass"               if bin_name == "plastic" else "plastic"
    other_pct    = bin_volumes.get(other_name, 0.0)
    other_label  = "Glass Bin (Right)"   if bin_name == "plastic" else "Plastic Bin (Left)"

    message = (
        f"[TIDEY ALERT] {bin_label} is {current_pct:.1f}% full — please empty it soon!\n"
        f"Other bin: {other_label} at {other_pct:.1f}%."
    )

    try:
        with open('config.json', 'r') as f:
            sms_cfg = json.load(f).get("sms", {})
    except Exception:
        sms_cfg = {}

    if _send_sms(message, sms_cfg):
        _sms_last_sent[bin_name] = now
# ------------------------

# --- WEATHER & TIDE MONITOR ---
# OWM weather IDs: 2xx=thunderstorm, 3xx=drizzle, 5xx=rain, 7xx=atmosphere, 800=clear, 80x=clouds
_RAIN_IDS    = set(range(200, 322)) | set(range(500, 532))
_BAD_WX_IDS  = set(range(200, 782))
_PH_TZ       = timezone(timedelta(hours=8))

live_weather = {
    "current":      {},
    "forecast_1h":  {},
    "tide":         {"is_peak": False, "next_high_in_min": None, "next_high_height": None, "enabled": False},
    "warnings":     {"rain": False, "bad_weather": False, "peak_tide": False},
    "last_updated": None,
}
_rain_alert_sent_at = 0.0
_tide_cache         = None
_tide_cache_time    = 0.0

def _fetch_owm_current(lat, lon, api_key):
    try:
        r = http_requests.get(
            "https://api.openweathermap.org/data/2.5/weather",
            params={"lat": lat, "lon": lon, "appid": api_key, "units": "metric"},
            timeout=15,
        )
        return r.json() if r.status_code == 200 else None
    except Exception as e:
        print(f"[WEATHER] OWM current error: {e}")
        return None

def _fetch_owm_forecast(lat, lon, api_key):
    try:
        r = http_requests.get(
            "https://api.openweathermap.org/data/2.5/forecast",
            params={"lat": lat, "lon": lon, "appid": api_key, "units": "metric", "cnt": 3},
            timeout=15,
        )
        return r.json() if r.status_code == 200 else None
    except Exception as e:
        print(f"[WEATHER] OWM forecast error: {e}")
        return None

def _fetch_stormglass_tides(lat, lon, api_key):
    try:
        r = http_requests.get(
            "https://api.stormglass.io/v2/tide/extremes/point",
            params={"lat": lat, "lng": lon},
            headers={"Authorization": api_key},
            timeout=15,
        )
        return r.json() if r.status_code == 200 else None
    except Exception as e:
        print(f"[TIDE] Stormglass error: {e}")
        return None

def _check_rain_alert(cond):
    global _rain_alert_sent_at
    now = time.time()
    if (now - _rain_alert_sent_at) < SMS_COOLDOWN_SECONDS:
        return
    try:
        with open('config.json', 'r') as f:
            sms_cfg = json.load(f).get("sms", {})
    except Exception:
        sms_cfg = {}
    msg = (
        f"⛈ TIDEY RAIN ALERT\n"
        f"Condition: {cond.get('weather_label', 'Rain')}\n"
        f"Rain: {cond.get('rain', 0):.1f}mm  |  Wind: {cond.get('wind_speed', 0):.0f} km/h\n"
        f"STOP the test now and bring TIDEY to shelter!"
    )
    if _send_sms(msg, sms_cfg):
        _rain_alert_sent_at = now

def _weather_monitor_loop():
    global live_weather, _tide_cache, _tide_cache_time
    while True:
        try:
            with open('config.json', 'r') as f:
                cfg = json.load(f)
            loc     = cfg.get("location", {"lat": 10.9000, "lon": 123.8928})
            wx_cfg  = cfg.get("weather", {})
            owm_key = wx_cfg.get("owm_api_key", "")
            sg_key  = wx_cfg.get("stormglass_api_key", "")
            lat, lon = loc["lat"], loc["lon"]

            # -- Current weather (OpenWeatherMap) --
            if owm_key:
                cur_data   = _fetch_owm_current(lat, lon, owm_key)
                fcast_data = _fetch_owm_forecast(lat, lon, owm_key)

                if cur_data:
                    wx     = cur_data.get("weather", [{}])[0]
                    wx_id  = int(wx.get("id", 800))
                    label  = wx.get("description", "unknown").capitalize()
                    rain   = float(cur_data.get("rain", {}).get("1h", 0))
                    wind   = float(cur_data.get("wind", {}).get("speed", 0)) * 3.6  # m/s → km/h
                    temp   = cur_data.get("main", {}).get("temp")
                    humid  = cur_data.get("main", {}).get("humidity")
                    is_rain = wx_id in _RAIN_IDS or rain > 0
                    is_bad  = wx_id in _BAD_WX_IDS

                    live_weather["current"] = {
                        "temperature":    temp,
                        "rain":           rain,
                        "wind_speed":     round(wind, 1),
                        "humidity":       humid,
                        "weather_code":   wx_id,
                        "weather_label":  label,
                        "is_raining":     is_rain,
                        "is_bad_weather": is_bad,
                    }
                    live_weather["warnings"]["rain"]        = is_rain
                    live_weather["warnings"]["bad_weather"] = is_bad
                    live_weather["last_updated"] = time.strftime("%H:%M")

                    if is_rain:
                        _check_rain_alert(live_weather["current"])

                # Next 3-hour forecast slot
                if fcast_data and fcast_data.get("list"):
                    nxt    = fcast_data["list"][1] if len(fcast_data["list"]) > 1 else fcast_data["list"][0]
                    fh_wx  = nxt.get("weather", [{}])[0]
                    live_weather["forecast_1h"] = {
                        "temperature":        nxt.get("main", {}).get("temp"),
                        "precipitation_prob": round(nxt.get("pop", 0) * 100),
                        "rain":               float(nxt.get("rain", {}).get("3h", 0)),
                        "weather_label":      fh_wx.get("description", "unknown").capitalize(),
                    }

            # -- Tides (Stormglass, cached 2 h) --
            now_ts = time.time()
            if sg_key and (now_ts - _tide_cache_time > 7200 or _tide_cache is None):
                _tide_cache      = _fetch_stormglass_tides(lat, lon, sg_key)
                _tide_cache_time = now_ts

            if _tide_cache and "data" in _tide_cache:
                now_ts  = time.time()
                is_peak = False
                nxt_min = None
                nxt_ht  = None
                for ex in _tide_cache["data"]:
                    if ex.get("type", "").lower() != "high":
                        continue
                    try:
                        ex_ts = datetime.fromisoformat(ex["time"]).timestamp()
                    except Exception:
                        continue
                    diff = ex_ts - now_ts
                    if abs(diff) < 1800:
                        is_peak = True
                    if diff > 0 and nxt_min is None:
                        nxt_min = round(diff / 60)
                        nxt_ht  = round(ex.get("height", 0), 2)

                live_weather["tide"] = {
                    "is_peak":          is_peak,
                    "next_high_in_min": nxt_min,
                    "next_high_height": nxt_ht,
                    "enabled":          True,
                }
                live_weather["warnings"]["peak_tide"] = is_peak
            else:
                live_weather["tide"]["enabled"] = bool(sg_key)

        except Exception as e:
            print(f"[WEATHER LOOP] {e}")

        time.sleep(600)   # refresh every 10 minutes
# ──────────────────────────────────────────────────────

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
robot_base.vision = eyes
robot_base.arm    = robot_arm
eyes.start_stream()
robot_base.start_sensor_watchdog()
threading.Thread(target=_weather_monitor_loop, daemon=True).start()

# 3. Web Routes
@app.route('/')
def index():
    # Pass the live configuration dictionary to the HTML template
    return render_template('index.html', 
                           grab_zone=eyes.zone_cfg, 
                           response_zone=eyes.response_cfg,
                           rt_left=eyes.rt_left,             # <--- UPDATED: Pass all 4 sides
                           rt_right=eyes.rt_right,
                           rt_top=eyes.rt_top,
                           rt_bottom=eyes.rt_bottom,
                           rt_angle=eyes.rt_angle)

def video_feed_generator():
    while eyes.stream_active:
        frame = eyes.get_frame()
        if frame:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(video_feed_generator(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/api/status', methods=['GET'])
def get_status():
    """Sends the latest chassis data and text status to the webpage."""
    global robot_status
    telemetry = robot_base.get_telemetry()
    telemetry["action"] = robot_status
    return jsonify(telemetry)

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
    if action == 'pickup_v':
        threading.Thread(target=robot_arm.pickup_sequence, args=('vertical',)).start()
    elif action == 'pickup_h':
        threading.Thread(target=robot_arm.pickup_sequence, args=('horizontal',)).start()
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
    boundary_mode = data.get('boundary_mode', False)
    grid_size = float(data.get('distance', 0))

    if boundary_mode:
        threading.Thread(target=robot_base.sweep_area).start()
        return jsonify({"status": "sweeping", "mode": "boundary"})

    if grid_size > 0:
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

# --- NEW: ROUTE FOR RECTANGULAR ZONE SLIDERS ---
# --- UPDATED: ROUTE FOR RECTANGULAR ZONE SLIDERS ---
@app.route('/cmd/vision/red_tape_limit', methods=['POST'])
def update_red_tape_limit():
    data = request.json
    
    # 1. Update live camera feed instantly
    if 'rt_left' in data: eyes.rt_left = int(data['rt_left'])
    if 'rt_right' in data: eyes.rt_right = int(data['rt_right'])
    if 'rt_top' in data: eyes.rt_top = int(data['rt_top'])
    if 'rt_bottom' in data: eyes.rt_bottom = int(data['rt_bottom'])
    if 'rt_angle' in data: eyes.rt_angle = int(data['rt_angle'])
    
    # 2. Permanently save the new settings to config.json
    try:
        with open('config.json', 'r') as f:
            full_config = json.load(f)
            
        if 'vision' not in full_config:
            full_config['vision'] = {}
            
        full_config['vision']['red_tape_zone'] = {
            "left": eyes.rt_left,
            "right": eyes.rt_right,
            "top": eyes.rt_top,
            "bottom": eyes.rt_bottom,
            "angle": eyes.rt_angle
        }
        
        with open('config.json', 'w') as f:
            json.dump(full_config, f, indent=2)
            
        print("[SERVER] Red Tape Zone permanently saved to config.json!")
    except Exception as e:
        print(f"[SERVER] Error saving to config: {e}")
        
    return jsonify({"status": "success"})

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
    if 'offset_x' in data:
        eyes.response_cfg['offset_x'] = int(data['offset_x'])
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


@app.route('/cmd/vision/grab_zone', methods=['POST'])
def update_grab_zone():
    data = request.json
    
    # 1. Instantly update the live camera feed in memory
    if 'width' in data: 
        eyes.zone_cfg['width'] = int(data['width'])
    if 'height' in data: 
        eyes.zone_cfg['height'] = int(data['height'])
    if 'offset_x' in data: 
        eyes.zone_cfg['offset_x'] = int(data['offset_x'])
    if 'offset_y' in data: 
        eyes.zone_cfg['offset_y'] = int(data['offset_y'])
    if 'angle' in data:
        eyes.zone_cfg['angle'] = int(data['angle'])
        
    # 2. Permanently save the new settings to config.json
    try:
        with open('config.json', 'r') as f:
            full_config = json.load(f)
            
        if 'vision' not in full_config:
            full_config['vision'] = {}
            
        full_config['vision']['grab_zone'] = eyes.zone_cfg
        
        with open('config.json', 'w') as f:
            json.dump(full_config, f, indent=2)
            
        print("[SERVER] Grab Zone permanently saved to config.json!")
    except Exception as e:
        print(f"[SERVER] Error saving to config: {e}")
        
    return jsonify({"status": "updated", "config": eyes.zone_cfg})

@app.route('/api/test_sms', methods=['POST'])
def test_sms():
    try:
        with open('config.json', 'r') as f:
            sms_cfg = json.load(f).get("sms", {})
    except Exception:
        sms_cfg = {}
    msg = (
        f"[TIDEY TEST] SMS notifications are working!\n"
        f"Plastic Bin: {bin_volumes['plastic']:.1f}% | Glass Bin: {bin_volumes['glass']:.1f}%"
    )
    ok = _send_sms(msg, sms_cfg)
    return jsonify({"status": "sent" if ok else "failed"})

@app.route('/api/weather', methods=['GET'])
def get_weather():
    return jsonify(live_weather)

@app.route('/api/bin_volumes', methods=['GET'])
def get_bin_volumes():
    return jsonify(bin_volumes)

@app.route('/api/bin_volumes/reset', methods=['POST'])
def reset_bin_volumes():
    data = request.json or {}
    target = data.get('bin', 'all')
    if target == 'plastic':
        bin_volumes['plastic'] = 0.0
    elif target == 'glass':
        bin_volumes['glass'] = 0.0
    else:
        bin_volumes['plastic'] = 0.0
        bin_volumes['glass'] = 0.0
    _save_bin_volumes(bin_volumes)
    print(f"[BIN] Reset '{target}' bin(s).")
    return jsonify({"status": "ok", "bin_volumes": bin_volumes})

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
    global tracking_active, robot_status
    robot_status = "Auto-Aligning..."
    print("[AUTO] Full-Containment Target Lock loop started.")
    
    while tracking_active:
        try:
            tx = eyes.target_x
            ty_top = getattr(eyes, 'target_y_top', None)
            ty_bottom = getattr(eyes, 'target_y_bottom', None)
            
            # Read orientation and class name from the camera
            t_orient = getattr(eyes, 'target_orientation', 'vertical')
            t_class = getattr(eyes, 'target_class', 'unknown') # Make sure vision.py sets self.target_class!
            
            # Safety Check
            if ty_top is None and hasattr(eyes, 'target_y'):
                print("[WARNING] Update vision.py to the Core Hitbox version.")
                tracking_active = False
                break
            
            if tx is not None and ty_top is not None and ty_bottom is not None:
                
                # --- RED BOX (GRAB ZONE) BOUNDARIES ---
                zh = eyes.zone_cfg.get("height", 90)
                oy = eyes.zone_cfg.get("offset_y", 0)
                
                red_center_y = 360 - (zh / 2) + oy
                red_top = red_center_y - (zh / 2)
                red_bottom = red_center_y + (zh / 2)
                
                crosshair_left = 260 
                crosshair_right = 380

                # ALIGNMENT: Keep the center of the trash in the lane
                if tx < crosshair_left:
                    robot_base.spin_left()
                elif tx > crosshair_right:
                    robot_base.spin_right()
                else:
                    # APPROACH STAGE
                    green_height = ty_bottom - ty_top
                    is_contained = (ty_top >= red_top) and (ty_bottom <= red_bottom)
                    is_giant_trash = (green_height >= zh) and (ty_bottom >= red_bottom)
                    
                    if is_contained or is_giant_trash:
                        robot_base.stop()
                        print(f"[AUTO] Target locked! Class: {t_class} | Orientation: {t_orient.upper()}. Initiating Sequence.")
                        
                        tracking_active = False 
                        # Update this section in main_server.py
                        def auto_pickup_and_drop(orientation, trash_class):
                            # 1. Execute Pickup
                            robot_arm.pickup_sequence(orientation)
                            
                            # Normalize the string to lowercase for easier comparison
                            safe_class = trash_class.lower() 
                            
                            # 2. Determine Drop Zone based on your specific class names
                            if safe_class == 'glass_bottles':
                                zone = 'r'
                            elif safe_class in ['general_plastic', 'plastic_bottles']:
                                zone = 'l'
                            else:
                                # Fallback to center if the class is unknown or Red_Tape
                                zone = 'c' 
                                
                            # 3. Execute Drop Off
                            robot_arm.return_sequence(zone)
                            record_bin_pickup(trash_class)
                            print(f"[AUTO] Dropped {trash_class} in zone: {zone}")
                                                    
                        # Run the combined sequence in the background thread
                        threading.Thread(target=auto_pickup_and_drop, args=(t_orient, t_class)).start()
                    else:
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