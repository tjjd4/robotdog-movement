import asyncio
import websockets
import json
from src.model.custom_types.index import BehaviorState

async def test_websocket():
    uri = "ws://localhost:8000/ws"
    
    async with websockets.connect(uri) as websocket:
        print("Connected to WebSocket server")
        
        # Test basic command structure
        basic_command = {
            "horizontal_velocity": [0.0, 0.0],
            "behavior_state": BehaviorState.REST.value,
            # "is_gyro_activated": False
        }
        
        # Test moving forward
        forward_command = basic_command.copy()
        forward_command["horizontal_velocity"] = [1.0, 0.0]
        forward_command["behavior_state"] = BehaviorState.MOVE.value
        
        print("\nTesting forward movement")
        await websocket.send(json.dumps(forward_command))
        response = await websocket.recv()
        print(f"Response: {response}")
        
        # Test turning left
        turn_left_command = basic_command.copy()
        turn_left_command["horizontal_velocity"] = [1.0, 0.0]
        turn_left_command["yaw_rate"] = 1.0
        turn_left_command["behavior_state"] = BehaviorState.MOVE.value
        
        print("\nTesting left turn")
        await websocket.send(json.dumps(turn_left_command))
        response = await websocket.recv()
        print(f"Response: {response}")
        
        # Test standing up
        stand_command = basic_command.copy()
        stand_command["behavior_state"] = BehaviorState.STAND.value
        
        print("\nTesting stand up")
        await websocket.send(json.dumps(stand_command))
        response = await websocket.recv()
        print(f"Response: {response}")
        
        # Test calibration
        calibrate_command = basic_command.copy()
        calibrate_command["behavior_state"] = BehaviorState.CALIBRATE.value
        
        print("\nTesting calibration")
        await websocket.send(json.dumps(calibrate_command))
        response = await websocket.recv()
        print(f"Response: {response}")
        
        # Test invalid command
        invalid_command = {"invalid": "command"}
        
        print("\nTesting invalid command")
        await websocket.send(json.dumps(invalid_command))
        response = await websocket.recv()
        print(f"Response: {response}")

if __name__ == "__main__":
    asyncio.run(test_websocket())