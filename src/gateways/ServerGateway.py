import json
import asyncio
import numpy as np
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse

from src.model.custom_types.index import MotionCommand
from src.model.Robotdog import Robotdog

class ServerGateway:
    def __init__(self, robotdog: Robotdog):
        self.app = FastAPI()
        self.robotdog = robotdog
        self._setup_routes()

    def _setup_routes(self):
        self.app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],
            allow_credentials=True,
            allow_methods=["*"],
            allow_headers=["*"],
        )

        @self.app.post("/command")
        async def receive_command(command: dict):
            self.robotdog.run_command_from_dict(command)
            return {"status": "received"}
        
        @self.app.websocket("/ws/control")
        async def control_robotdog(websocket: WebSocket):
            await websocket.accept()
            try:
                while True:
                    data = await websocket.receive_text()
                    try:
                        print(f"[CONTROL] Received data: {data}")
                        command = MotionCommand(**json.loads(data))
                        self.robotdog.run(command)
                        await websocket.send_text("Command executed successfully")
                    except Exception as e:
                        await websocket.send_text(f"Error executing command: {str(e)}")
            except WebSocketDisconnect:
                print("[CONTROL] WebSocket disconnected")

        @self.app.post("/lidar/start")
        def start_lidar():
            self.robotdog.start_lidar()
            return {"status": "Lidar started"}

        @self.app.post("/lidar/stop")
        def stop_lidar():
            self.robotdog.stop_lidar()
            return {"status": "Lidar stopped"}
        

        @self.app.websocket("/ws/pointcloud")
        async def lidar_stream(websocket: WebSocket):
            await websocket.accept()
            try:
                while True:
                    distances = self.robotdog.get_latest_scan()
                    # Convert distances to 2D Cartesian points (x, y)
                    points = [
                        {
                            "x": float(d * np.cos(np.radians(angle))),
                            "y": float(d * np.sin(np.radians(angle)))
                        }
                        for angle, d in enumerate(distances)
                        if d is not None
                    ]
                    await websocket.send_text(json.dumps(points))
                    await asyncio.sleep(0.2)
            except WebSocketDisconnect:
                print("[LIDAR] WebSocket disconnected")

        @self.app.get("/lidar/stream")
        async def lidar_stream():
            return StreamingResponse(
                self.robotdog.get_lidar_stream(),
                media_type="multipart/x-mixed-replace; boundary=frame"
            )


        @self.app.post("/camera/start")
        def start_camera():
            self.robotdog.start_camera()
            return {"status": "Camera started"}

        @self.app.post("/camera/stop")
        def stop_camera():
            self.robotdog.stop_camera()
            return {"status": "Camera stopped"}

        @self.app.post("/detection/start")
        def start_detection():
            self.robotdog.start_detection()
            return {"status": "Detection started"}

        @self.app.post("/detection/stop")
        def stop_detection():
            self.robotdog.stop_detection()
            return {"status": "Detection stopped"}

        @self.app.get("/camera/video")
        async def video_stream():
            return StreamingResponse(
                self.robotdog.get_camera_stream(),
                media_type="multipart/x-mixed-replace; boundary=frame"
            )

    def get_app(self) -> FastAPI:
        return self.app
