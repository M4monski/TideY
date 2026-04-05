import time
import threading
import cv2
import numpy as np
from picamera2 import Picamera2
from ultralytics import YOLO

class VisionSystem:
    def __init__(self, config):
        print("[VISION] Loading YOLOv8 model into memory... please wait.")
        model_path = config.get("model_path", "best.pt")
        self.model = YOLO(model_path)
        
        self.zone_cfg = config.get("grab_zone", {
            "width": 120,   
            "height": 90,   
            "offset_x": 0,  
            "offset_y": -25,  
            "angle": 0      
        })

        self.response_cfg = config.get("response_zone", {
            "bottom_width": 400, 
            "top_width": 150,    
            "height": 180,       
            "offset_y": 120      
        })
        
        self.picam2 = Picamera2()
        self.camera_lock = threading.Lock()
        
        self.current_stream_frame = None
        self.stream_active = False
        self.frame_ready = threading.Event()
        
        # --- NEW: Holds the X and Y coordinates for the Core Hitbox ---
        self.target_x = None 
        self.target_y_top = None
        self.target_y_bottom = None
        
    def start_stream(self):
        self.stream_active = True
        with self.camera_lock:
            self.picam2.configure(self.picam2.create_video_configuration(main={"size": (1920, 1080)}))
            self.picam2.start()
            
        self.stream_thread = threading.Thread(target=self._generate_mjpeg_frames, daemon=True)
        self.stream_thread.start()
        print("[VISION] Live stream started.")

    def _generate_mjpeg_frames(self):
        while self.stream_active:
            frame_to_process = None
            try:
                if self.camera_lock.acquire(blocking=False):
                    try:
                        frame_to_process = self.picam2.capture_array()
                    finally:
                        self.camera_lock.release()
                
                if frame_to_process is not None:
                    frame_to_process = cv2.resize(frame_to_process, (640, 360))

                    if frame_to_process.shape[-1] == 4:
                        frame_to_process = frame_to_process[:, :, :3]
                    frame_to_process = cv2.cvtColor(frame_to_process, cv2.COLOR_RGB2BGR)

                    results = self.model(frame_to_process, conf=0.50, verbose=False)
                    annotated_frame = results[0].plot()
                    
                    # --- EXTRACT THE SMALLER MIDDLE BOX (CORE HITBOX) ---
                    boxes = results[0].boxes
                    best_x = None
                    core_top = None
                    core_bottom = None
                    max_c = 0
                    
                    for box in boxes:
                        c = float(box.conf[0])
                        if c > max_c:
                            max_c = c
                            x1, y1, x2, y2 = box.xyxy[0].tolist()
                            best_x = (x1 + x2) / 2 # Center X for steering
                            
                            # Math to create the smaller middle box (middle 50%)
                            box_width = x2 - x1
                            box_height = y2 - y1
                            
                            core_left = x1 + (box_width * 0.25)
                            core_right = x2 - (box_width * 0.25)
                            core_top = y1 + (box_height * 0.25)
                            core_bottom = y2 - (box_height * 0.25)
                            
                            # Draw the Green Hitbox on the live feed!
                            cv2.rectangle(annotated_frame, (int(core_left), int(core_top)), (int(core_right), int(core_bottom)), (0, 255, 0), 2)
                            cv2.putText(annotated_frame, "CORE HITBOX", (int(core_left), int(core_top) - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 2)
                            
                    self.target_x = best_x
                    self.target_y_top = core_top
                    self.target_y_bottom = core_bottom
                    # ----------------------------------------------------
                    
                    # Draw Grab Zone
                    zw = self.zone_cfg["width"]
                    zh = self.zone_cfg["height"]
                    base_center_x = 640 // 2
                    original_center_y = 360 - (zh // 2)
                    new_center_x = base_center_x + self.zone_cfg["offset_x"]
                    new_center_y = original_center_y + self.zone_cfg["offset_y"]
                    rotated_rect = ((new_center_x, new_center_y), (zw, zh), self.zone_cfg["angle"])
                    box_points = np.int32(cv2.boxPoints(rotated_rect))
                    cv2.drawContours(annotated_frame, [box_points], 0, (0, 0, 255), 2)
                    top_y = np.min(box_points[:, 1])
                    left_x = np.min(box_points[:, 0])
                    cv2.putText(annotated_frame, "GRAB ZONE", (left_x, top_y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                    # Draw Response Zone
                    rbw = self.response_cfg["bottom_width"]
                    rtw = self.response_cfg["top_width"]
                    rh = self.response_cfg["height"]
                    base_y = (360 // 2) + self.response_cfg["offset_y"]
                    resp_top_y = base_y - rh
                    bottom_left = (base_center_x - (rbw // 2), base_y)
                    bottom_right = (base_center_x + (rbw // 2), base_y)
                    top_left = (base_center_x - (rtw // 2), resp_top_y)
                    top_right = (base_center_x + (rtw // 2), resp_top_y)
                    cv2.line(annotated_frame, bottom_left, top_left, (255, 0, 0), 2)
                    cv2.line(annotated_frame, bottom_right, top_right, (255, 0, 0), 2)
                    cv2.line(annotated_frame, top_left, top_right, (255, 0, 0), 2)
                    cv2.putText(annotated_frame, "RESPONSE ZONE", (top_left[0], resp_top_y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

                    # Compress
                    ret, buffer = cv2.imencode('.jpg', annotated_frame)
                    if ret:
                        self.current_stream_frame = buffer.tobytes()
                        self.frame_ready.set()
                        self.frame_ready.clear()
                else:
                    time.sleep(0.05)
            except Exception as e:
                print(f"[VISION] Stream error: {e}")
                time.sleep(0.1)

    def get_frame(self):
        self.frame_ready.wait()
        return self.current_stream_frame

    def capture_high_res(self, filepath):
        with self.camera_lock:
            self.picam2.stop()
            self.picam2.configure(self.picam2.create_still_configuration(main={"size": (1920, 1080)}))
            self.picam2.start()
            self.picam2.capture_file(filepath)
            self.picam2.stop()
            self.picam2.configure(self.picam2.create_video_configuration(main={"size": (1920, 1080)}))
            self.picam2.start()

    def stop(self):
        self.stream_active = False
        if hasattr(self, 'stream_thread'):
            self.stream_thread.join(timeout=2)
        self.picam2.stop()
        self.picam2.close()
        print("[VISION] Camera shut down.")