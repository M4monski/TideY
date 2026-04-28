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
        
        # --- Load saved Red Tape Zone from config ---
        rt_cfg = config.get("red_tape_zone", {
            "left": 0,
            "right": 640,
            "top": 180,
            "bottom": 230,
            "angle": 0
        })
        self.rt_left = rt_cfg.get("left", 0)
        self.rt_right = rt_cfg.get("right", 640)
        self.rt_top = rt_cfg.get("top", 180)
        self.rt_bottom = rt_cfg.get("bottom", 230)
        self.rt_angle = rt_cfg.get("angle", 0)
        
        self.red_tape_triggered = False
        self.tape_detection_active = True  
        
        self.picam2 = Picamera2()
        self.camera_lock = threading.Lock()
        
        self.current_stream_frame = None
        self.stream_active = False
        self.frame_ready = threading.Event()
        
        self.target_x = None 
        self.target_y_top = None
        self.target_y_bottom = None
        self.target_orientation = "vertical" 
        self.target_class = "unknown" # <--- NEW: Initialize target class
        
        # --- NEW: Auto-Pickup Triggers ---
        self.target_in_response_zone = False 
        self.target_in_grab_zone = False      
        
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
                    
                    boxes = results[0].boxes
                    best_x = None
                    core_top = None
                    core_bottom = None
                    current_orientation = "vertical" 
                    current_class = "unknown" # <--- NEW: Reset class every frame
                    max_c = 0
                    
                    # --- NEW: Reset triggers every frame ---
                    self.target_in_response_zone = False 
                    self.target_in_grab_zone = False     
                    
                    tapes_hitting_boundary = 0
                    large_tape_detected = False 
                    
                    # --- Pre-calculate the rotated rectangle contour points ---
                    rt_w = self.rt_right - self.rt_left
                    rt_h = self.rt_bottom - self.rt_top
                    rt_cx = self.rt_left + rt_w // 2
                    rt_cy = self.rt_top + rt_h // 2
                    
                    rt_rotated_rect = ((rt_cx, rt_cy), (rt_w, rt_h), self.rt_angle)
                    rt_box_points = np.int32(cv2.boxPoints(rt_rotated_rect))
                    
                    for box in boxes:
                        c = float(box.conf[0])
                        
                        cls_idx = int(box.cls[0])
                        cls_name = self.model.names[cls_idx]
                        
                        # --- ROTATED RECTANGLE INTERSECTION CHECK ---
                        if cls_name == "Red_Tape":
                            tx1, ty1, tx2, ty2 = box.xyxy[0].tolist()
                            tape_w = tx2 - tx1  
                            
                            tx_c = (tx1 + tx2) / 2.0
                            ty_c = (ty1 + ty2) / 2.0
                            
                            dist = cv2.pointPolygonTest(rt_box_points, (tx_c, ty_c), False)
                            
                            if dist >= 0:
                                tapes_hitting_boundary += 1
                                
                                if tape_w >= (rt_w * 0.27):
                                    large_tape_detected = True

                        # Target tracking logic (ignores Red_Tape)
                        if c > max_c and cls_name != "Red_Tape":
                            max_c = c
                            current_class = cls_name # <--- NEW: Store the name of the trash detected
                            x1, y1, x2, y2 = box.xyxy[0].tolist()
                            best_x = (x1 + x2) / 2 
                            
                            box_width = x2 - x1
                            box_height = y2 - y1
                            
                            if box_width > (box_height * 1.3):
                                current_orientation = "horizontal"
                            else:
                                current_orientation = "vertical"
                            
                            core_left = x1 + (box_width * 0.25)
                            core_right = x2 - (box_width * 0.25)
                            core_top = y1 + (box_height * 0.25)
                            core_bottom = y2 - (box_height * 0.25)
                            
                            # Calculate zone boundaries based on your existing config math
                            resp_base_y = (360 // 2) + self.response_cfg["offset_y"]
                            resp_top_y = resp_base_y - self.response_cfg["height"]
                            
                            grab_center_y = (360 - (self.zone_cfg["height"] // 2)) + self.zone_cfg["offset_y"]
                            grab_top_y = grab_center_y - (self.zone_cfg["height"] // 2)
                            grab_bottom_y = grab_center_y + (self.zone_cfg["height"] // 2)

                            # If the bottom of the trash is inside the response zone
                            if resp_top_y < y2 < resp_base_y:
                                self.target_in_response_zone = True
                                
                            # If the bottom of the trash touches the grab zone
                            if grab_top_y < y2 < grab_bottom_y:
                                self.target_in_grab_zone = True
                            
                            cv2.rectangle(annotated_frame, (int(core_left), int(core_top)), (int(core_right), int(core_bottom)), (0, 255, 0), 2)
                            cv2.putText(annotated_frame, f"CORE: {current_orientation.upper()} | {current_class}", (int(core_left), int(core_top) - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 2)
                            
                    self.target_x = best_x
                    self.target_y_top = core_top
                    self.target_y_bottom = core_bottom
                    self.target_orientation = current_orientation
                    self.target_class = current_class # <--- NEW: Export to the main server

                    # --- UPDATED: LIVE CONTINUOUS TAPE TRIGGER LOGIC W/ BLINDFOLD ---
                    if self.tape_detection_active:
                        if tapes_hitting_boundary >= 50 or large_tape_detected:
                            self.red_tape_triggered = True
                            
                            trigger_reason = "LARGE TAPE DETECTED!" if large_tape_detected else "5+ TAPES IN ZONE!"
                            cv2.putText(annotated_frame, f"TURN AROUND: {trigger_reason}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 3)
                        else:
                            self.red_tape_triggered = False
                            if tapes_hitting_boundary > 0:
                                cv2.putText(annotated_frame, f"TAPES IN ZONE: {tapes_hitting_boundary}/5", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 165, 255), 2)
                    else:
                        # Force the flag to stay off while turning/picking up, and put a visual indicator on the stream
                        self.red_tape_triggered = False
                        cv2.putText(annotated_frame, "IGNORING TAPE (MANEUVERING)", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 255), 2)

                    
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

                    # --- Draw the Rotated Rectangular Tape Zone ---
                    cv2.drawContours(annotated_frame, [rt_box_points], 0, (0, 255, 255), 2)
                    
                    rt_top_y = np.min(rt_box_points[:, 1])
                    rt_left_x = np.min(rt_box_points[:, 0])
                    cv2.putText(annotated_frame, "TAPE BOUNDARY ZONE", (rt_left_x, rt_top_y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

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

    # --- NEW CONTROL METHODS ---
    def pause_tape_detection(self):
        """Disables red tape triggering during maneuvers."""
        self.tape_detection_active = False
        self.red_tape_triggered = False 

    def resume_tape_detection(self):
        """Re-enables red tape triggering for straight driving."""
        self.tape_detection_active = True

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