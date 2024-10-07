import cv2
import time

def scale_vel_cmd(cmd_val, scale=0.5):
    '''
    Apply Proportional Gain Constant
    '''
    return scale * cmd_val

def cb_cmd_vel(drone, continuous_action):
    '''
    Control the drone using the continuous action values
    '''
    linear_z, linear_x, linear_y, angular_z = continuous_action[0]

    drone.set_pitch(scale_vel_cmd(linear_y))
    drone.set_roll(scale_vel_cmd(linear_x))
    drone.set_yaw(scale_vel_cmd(angular_z))
    drone.set_throttle(scale_vel_cmd(linear_z))

def drone_land(drone):
    '''
    Perform drone landing sequence
    '''
    cb_cmd_vel(drone, [[0, 0, 0, 0]])
    drone.land()
    time.sleep(5)
    drone.quit()
    cv2.destroyAllWindows()

def handler(event, sender, data, **args):
    '''
    TelloPy handler function
    '''
    drone = sender
    if event is drone.EVENT_FLIGHT_DATA:
        pass