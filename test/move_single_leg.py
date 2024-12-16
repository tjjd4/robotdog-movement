import time
import bezier
import numpy as np

from move_logic.quadruped import Robotdog, Motor

def controller(momentum):
    momentum[:3] = [0, 0, 1]
    return momentum

def move_single_leg(self, controller=None, shoulder=Motor.FR_SHOULDER, elbow=Motor.FR_ELBOW, hip=Motor.FR_HIP):
    """
    Walks around based on the controller inputted momentum
    :param controller: the controller that is called to determine the robot momentum
    :returns: None, enters an infinite loop 
    """
    momentum = np.asarray([0,0,1,0],dtype=np.float32)  # 前三個值 x(前後), z(左右), y(上下) 為步伐大小的縮放值, 第四個值不為 0 時結束動作
    index = 0
    
    # Generate footstep
    s_vals = np.linspace(0.0, 1.0, 20)  # from 0 to 1, sperate to 20 point
    step_nodes = np.asfortranarray([  # define the curve using these four points -> bezier curve
        [-1.0, -1.0, 1.0, 1.0],
        [-1.0, -1.0, 1.0, 1.0],
        [-15.0, -10, -10, -15.0],
    ])
    curve = bezier.Curve(step_nodes, degree=3)
    step = curve.evaluate_multi(s_vals)
    slide_nodes = np.asfortranarray([
        [1.0, -1.0],
        [1.0, -1.0],
        [-15.0, -15],
    ])
    curve = bezier.Curve(slide_nodes, degree=1)
    slide = curve.evaluate_multi(s_vals)

    motion = np.concatenate((step,slide), axis=1)

    close = False
    while not close:
        momentum = controller(momentum)
        tragectory = motion * momentum[:3, None]
        if momentum[3]:
            close = True
        x,z,y = tragectory
        # 
        i1 = index%40
        i2 = (index+20)%40 
        # Apply movement based movement
        self.inverse_positioning(shoulder,elbow,x[i1],y[i1]-1,z=z[i1],hip=hip,right=True)
        index += 1

Robotdog.move = move_single_leg()


if __name__ == '__main__':
    robotdog = Robotdog()
    
    print("Calibrating robot dog to default position...")
    robotdog.calibrate()
    action = input("press 'Enter' to -> Standup")
    robotdog.calibrate_by_inverse_positioning()
    action = input("press 'Enter' to start ->")

    momentum = np.asarray([0, 0, 1, 0], dtype=np.float32)  # 最後一個值為 0 表示不關閉

    print("Starting robot dog movement. Press Ctrl+C to stop.")
    try:
        robotdog.move(controller)  # 使用 lambda 傳遞 momentum 給 controller
    except KeyboardInterrupt:
        print("\nCtrl+C detected! Sending stop signal to robot dog...")

        momentum[3] = 1
        time.sleep(1)
        robotdog.calibrate()
        print("Movement stopped by user. Exiting program.")