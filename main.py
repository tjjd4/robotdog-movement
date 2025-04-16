from gateways.ServerGateway import ServerGateway
from model.Robotdog import Robotdog

robotdog = Robotdog()

gateway = ServerGateway(robotdog)
app = gateway.get_app()

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)