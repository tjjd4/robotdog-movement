from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
import json
import asyncio
from model.custom_types.index import MotionCommand
from model.Robotdog import Robotdog

class ServerGateway:
    def __init__(self, robotdog: Robotdog, lidar_controller):
        self.app = FastAPI()
        self.robotdog = robotdog
        self.lidar_controller = lidar_controller
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


        @self.app.websocket("/ws/pointcloud")
        async def lidar_stream(websocket: WebSocket):
            await websocket.accept()
            try:
                while True:
                    points = self.lidar_controller.get_latest_point_cloud()
                    await websocket.send_text(json.dumps(points.tolist()))
                    await asyncio.sleep(0.2)
            except WebSocketDisconnect:
                print("[LIDAR] WebSocket disconnected")


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
