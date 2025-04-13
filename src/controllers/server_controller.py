from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from model.Robotdog import Robotdog
from model.custom_types.index import MotionCommand
import json

app = FastAPI()
robot = Robotdog()

# Configure CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.get("/")
async def root():
    return {"message": "Robotdog API is running"}

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    
    try:
        while True:
            data = await websocket.receive_text()
            try:
                print(f"Received data: {data}")
                command = MotionCommand(**json.loads(data))
                print(f"Received command: {command}")
                robot.run(command)
                await websocket.send_text("Command executed successfully")
            except Exception as e:
                await websocket.send_text(f"Error executing command: {str(e)}")
                
    except WebSocketDisconnect:
        print("Client disconnected")