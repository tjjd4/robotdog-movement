import sys
import os

# Add src to path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

from fastapi import FastAPI
from fastapi.responses import StreamingResponse
from model.CameraController import CameraController
from model.YoloV11Detector import YoloV11Detector
from model.YoloWorldDetector import YoloWorldDetector

app = FastAPI()

# Initialize detectors
yolo_v11 = YoloV11Detector('yolo11n.pt')
yolo_world = YoloWorldDetector('yolov8s-world.pt')

# Initialize camera controller
camera_controller = CameraController()

@app.on_event("startup")
async def startup_event():
    camera_controller.start_camera()

@app.on_event("shutdown")
async def shutdown_event():
    camera_controller.stop_camera()

@app.get('/video')
async def video_feed():
    return StreamingResponse(
        camera_controller.generate_frames(),
        media_type='multipart/x-mixed-replace; boundary=frame'
    )

@app.post('/start_detection')
async def start_detection():
    camera_controller.start_detection()
    return {"status": "detection started"}

@app.post('/stop_detection')
async def stop_detection():
    camera_controller.stop_detection()
    return {"status": "detection stopped"}

@app.post('/set_detector/{detector_type}')
async def set_detector(detector_type: str):
    if detector_type.lower() == 'yolov11':
        camera_controller.set_detector(yolo_v11)
        return {"status": f"detector set to YoloV11"}
    elif detector_type.lower() == 'yoloworld':
        camera_controller.set_detector(yolo_world)
        return {"status": f"detector set to YoloWorld"}
    else:
        return {"error": "Invalid detector type. Use 'yolov11' or 'yoloworld'"}

@app.get('/status')
async def get_status():
    detector_status = None
    detector_type = None
    
    if camera_controller.detector:
        detector_status = camera_controller.detector.is_active()
        detector_type = type(camera_controller.detector).__name__
    
    return {
        "camera_active": camera_controller.is_active(),
        "detector_type": detector_type,
        "detector_active": detector_status
    }

if __name__ == '__main__':
    import uvicorn
    uvicorn.run(app, host='0.0.0.0', port=5000)