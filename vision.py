import time
import threading
import cv2
import numpy as np
from picamera2 import Picamera2
from ultralytics import YOLO

class VisionSystem:
    def __init__(self, config):
        print("[VISION] Loading YOLOv8 model into memory... please wait.")
        # Default to 'best.pt' if config is missing
        model_path = config.get("model_path", "best.pt")
        self.model = YOLO(model_path)
        
        # Load grab zone coordinates from config
        self.zone_cfg = config.get("grab_zone", {
            "width": 120, "height": 90, 
            "offset_x": -67, "offset_y": -120, "angle": 20
        })
        
        self.picam2 = Picamera2()
        self.camera_lock = threading.Lock()
        
        self.current_stream_frame = None
        self.stream_active = False
        self.frame_ready = threading.Event()
        
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
                    # 1. Resize for fast inference
                    frame_to_process = cv2.resize(frame_to_process, (640, 360))

                    # 2. Color correction
                    if frame_to_process.shape[-1] == 4:
                        frame_to_process = frame_to_process[:, :, :3]
                    frame_to_process = cv2.cvtColor(frame_to_process, cv2.COLOR_RGB2BGR)

                    # 3. YOLO Inference
                    results = self.model(frame_to_process, conf=0.50, verbose=False)
                    annotated_frame = results[0].plot()
                    
                    # 4. Draw Grab Zone from Config
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
                    cv2.putText(annotated_frame, "GRAB ZONE", (left_x, top_y - 8), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                    # 5. Compress
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
            
            res = self.model(filepath, conf=0.15)
            annotated_high_res = res[0].plot()
            cv2.imwrite(filepath, annotated_high_res)

            self.picam2.stop()
            
            # Restart stream
            self.picam2.configure(self.picam2.create_video_configuration(main={"size": (1920, 1080)}))
            self.picam2.start()

    def stop(self):
        self.stream_active = False
        if hasattr(self, 'stream_thread'):
            self.stream_thread.join(timeout=2)
        self.picam2.stop()
        self.picam2.close()
        print("[VISION] Camera shut down.")