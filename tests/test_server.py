import json
import time
import random
import asyncio
import numpy as np
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse
from io import BytesIO
from PIL import Image, ImageDraw

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

fake_lidar_running = False

@app.post("/command")
async def receive_command(command: dict):
    print("[COMMAND] Received:", command)
    return {"status": "received"}

@app.websocket("/ws/control")
async def control_robotdog(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            data = await websocket.receive_text()
            print("[CONTROL] Received:", data)
            await websocket.send_text("Command received")
    except WebSocketDisconnect:
        print("[CONTROL] WebSocket disconnected")

@app.post("/lidar/start")
def start_lidar():
    print("[LIDAR] Started")
    return {"status": "Lidar started"}

@app.post("/lidar/stop")
def stop_lidar():
    print("[LIDAR] Stopped")
    return {"status": "Lidar stopped"}

@app.websocket("/ws/pointcloud")
async def lidar_stream(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            if not fake_lidar_running:
                await asyncio.sleep(0.2)
                continue

            points = [
                {
                    "x": float(np.random.uniform(-500, 500)),
                    "y": float(np.random.uniform(-500, 500))
                }
                for _ in range(100)
            ]
            await websocket.send_text(json.dumps(points))
            await asyncio.sleep(0.2)
    except WebSocketDisconnect:
        print("[LIDAR] WebSocket disconnected")

@app.post("/camera/start")
def start_camera():
    print("[CAMERA] Started")
    return {"status": "Camera started"}

@app.post("/camera/stop")
def stop_camera():
    print("[CAMERA] Stopped")
    return {"status": "Camera stopped"}

@app.get("/camera/video")
async def video_stream():
    def gen():
        while True:
            # 產生隨機背景顏色
            r = random.randint(0, 255)
            g = random.randint(0, 255)
            b = random.randint(0, 255)
            img = Image.new("RGB", (640, 480), (r, g, b))
            draw = ImageDraw.Draw(img)

            # 畫時間戳（測試是否動態）
            timestamp = time.strftime("%H:%M:%S", time.localtime())
            draw.text((10, 10), f"Time: {timestamp}", fill="white")

            # 畫一個移動的圓圈（讓畫面有動態變化）
            t = time.time()
            x = int(320 + 100 * random.uniform(-1, 1))
            y = int(240 + 100 * random.uniform(-1, 1))
            draw.ellipse((x - 10, y - 10, x + 10, y + 10), fill="white")

            # 編碼成 JPEG
            buf = BytesIO()
            img.save(buf, format='JPEG')
            frame = buf.getvalue()

            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
            )
            time.sleep(0.1)

    return StreamingResponse(gen(), media_type="multipart/x-mixed-replace; boundary=frame")

@app.post("/detection/start")
def start_detection():
    print("[DETECTION] Started")
    return {"status": "Detection started"}

@app.post("/detection/stop")
def stop_detection():
    print("[DETECTION] Stopped")
    return {"status": "Detection stopped"}


