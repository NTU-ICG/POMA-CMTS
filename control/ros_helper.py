import numpy as np

def update_position(msg, position:np.array, toUpdate:bool=True, relative:bool=False, position_drone:np.array=None):
    '''
    Update position variable (type: pose.position)
    through ROS & vrpn_client_ros
    '''
    if toUpdate:
        value = msg.pose.position
        position[0:3] = [value.x, value.z, value.y]

        if relative:
            position[0:3] -= position_drone[0]

def update_orientation(msg, orientation:np.array):
    '''
    Update orientation variable (type: pose.orientation)
    through ROS & vrpn_client_ros
    '''
    value = msg.pose.orientation
    orientation[0:4] = [value.x, value.y, value.z, value.w]

def update_all(msg, position:np.array, orientation:np.array, toUpdate:bool=True, relative:bool=False):
    '''
    Update position and orientation variables simultaneously
    through ROS & vrpn_client_ros
    '''
    update_position(msg, position, toUpdate, relative)
    update_orientation(msg, orientation)

