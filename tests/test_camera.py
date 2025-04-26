from fastapi import FastAPI, Response
from fastapi.responses import StreamingResponse
from picamera2 import Picamera2
import cv2
from ultralytics import YOLO

app = FastAPI()

# Initialize the Raspberry Pi camera
camera = Picamera2()
camera.configure(camera.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
camera.preview_configuration.align()
camera.configure("preview")
camera.start()

# Load YOLO model (Ensure yolo___.pt exists at this path if using self fine-tune model)
model = YOLO("yolo11n_ncnn_model")

def generate_frames():
    while True:
        frame = camera.capture_array()

        # Convert to RGB (YOLO requires RGB format)
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        results = model(frame_rgb)

        # Annotate frame with detections
        annotated_frame = results[0].plot()

        # Get inference time and calculate FPS
        inference_time = results[0].speed.get('inference', 0)
        fps = 1000.0 / inference_time if inference_time > 0 else 0
        text = f'FPS: {fps:.1f}'

        # Draw FPS on the frame
        font = cv2.FONT_HERSHEY_SIMPLEX
        text_size = cv2.getTextSize(text, font, 1, 2)[0]
        text_x = annotated_frame.shape[1] - text_size[0] - 10  # Right-aligned
        text_y = text_size[1] + 10  # Top-aligned
        cv2.putText(annotated_frame, text, (text_x, text_y), font, 1, (255, 255, 255), 2, cv2.LINE_AA)

        ret, buffer = cv2.imencode('.jpg', annotated_frame)
        frame_jpeg = buffer.tobytes()

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_jpeg + b'\r\n')

@app.get('/video')
async def video_feed():
    return StreamingResponse(
        generate_frames(),
        media_type='multipart/x-mixed-replace; boundary=frame'
    )

if __name__ == '__main__':
    import uvicorn
    uvicorn.run(app, host='0.0.0.0', port=5000)