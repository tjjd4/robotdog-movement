from fastapi import FastAPI
from fastapi.responses import StreamingResponse, JSONResponse
from src.model.LidarController import LidarController

app = FastAPI()
lidar_controller = LidarController()

@app.on_event("startup")
def on_startup():
    print("[FastAPI] App started")

@app.on_event("shutdown")
def on_shutdown():
    print("[FastAPI] App shutting down")
    if lidar_controller.is_running():
        lidar_controller.stop()

@app.get("/lidar/start")
def start_lidar():
    if not lidar_controller.is_running():
        lidar_controller.start()
        return JSONResponse(content={"status": "started"})
    return JSONResponse(content={"status": "already running"})

@app.get("/lidar/stop")
def stop_lidar():
    if lidar_controller.is_running():
        lidar_controller.stop()
        return JSONResponse(content={"status": "stopped"})
    return JSONResponse(content={"status": "not running"})

@app.get("/lidar/status")
def lidar_status():
    return {"running": lidar_controller.is_running()}

@app.get("/lidar/stream")
def lidar_stream():
    if not lidar_controller.is_running():
        return JSONResponse(status_code=400, content={"error": "Lidar is not running"})
    return StreamingResponse(
        lidar_controller.generate_frames(),
        media_type="multipart/x-mixed-replace; boundary=frame"
    )
