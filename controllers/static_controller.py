# Note that this library requires sudo privileges

def controller(momentum):
    """
    Update the momentum of the robot based on keyboard presses.
    :param momentum: the existing momentum parameter
    :param accel: how quickly the robot starts walking in a given direction
    :param bound: the max/min magnitude that the robot can walk at
    :returns: updated momentum array
    """
    return momentum

if __name__ == "__main__":
    import numpy as np

    momentum = np.asarray([0,0,1],dtype=np.float32)
    while True:
        momentum = controller(momentum)
        print(momentum)