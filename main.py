from gateways.ServerGateway import ServerGateway
from model.Robotdog import Robotdog
from model.LidarController import LidarController

robotdog = Robotdog()
lidar_controller = LidarController()

gateway = ServerGateway(robotdog, lidar_controller)
app = gateway.get_app()

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)