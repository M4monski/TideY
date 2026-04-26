import cv2
from flask import Flask, Response

app = Flask(__name__)

def generate_frames():
    # Change to 1 or -1 if 0 doesn't find your USB camera
    camera = cv2.VideoCapture(2)
    
    while True:
        success, frame = camera.read()
        if not success:
            break 
        else:
            ret, buffer = cv2.imencode('.jpg', frame)
            frame_bytes = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

@app.route('/')
def index():
    return '''
    <html>
        <head><title>Camera Test</title></head>
        <body style="background: black; color: white; text-align: center;">
            <h2>Live Camera Feed</h2>
            <img src="/video_feed" style="max-width: 100%; border: 2px solid white;">
        </body>
    </html>
    '''

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == "__main__":
    print("\n--- Camera Test Stream Active ---")
    print("Open your browser and go to: http://<YOUR_PI_IP_ADDRESS>:5000")
    print("Press Ctrl+C to stop.\n")
    # 0.0.0.0 allows access from any device on the network
    app.run(host='0.0.0.0', port=5000, debug=False)