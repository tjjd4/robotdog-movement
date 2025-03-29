from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
import json
import asyncio
from model.custom_types.index import MotionCommand

class ServerGateway:
    def __init__(self, robotdog, lidar_controller):
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

    def get_app(self) -> FastAPI:
        return self.app
