#!/usr/bin/python3

import numpy as np
import cv2
import onnxruntime as ort

import sys
import time
import traceback

import av
import tellopy

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32MultiArray

from functools import partial

from helper_functions import *

## Global Variable ##
position_drone = np.zeros((1, 3), dtype=np.float32)
orientation_drone = np.zeros((1, 4), dtype=np.float32)

position_goal = np.zeros((1, 5, 3), dtype=np.float32)

obs_0_data = np.zeros((1, 5, 3), dtype=np.float32)
obs_2_data = np.zeros((1, 14), dtype=np.float32)

telloIDs = ["tello1", "tello2", "tello3"]
goalIDs = ["goal1", "goal2", "goal3"]

target_status = [1, 1, 0, 0, 0] # 1 if available, else 0

for i in len(target_status):
    if not target_status[i]:
        position_goal[0][i][0:3] = [float('inf'), float('inf'), float('inf')]

frame_count = 0

## ONNX Model for Drone Decision Making ##
model = "./VisualDrone.onnx"
sess = ort.InferenceSession(model, providers=["CUDAExecutionProvider"])

obs_0 = sess.get_inputs()[0].name
obs_1 = sess.get_inputs()[1].name
obs_2 = sess.get_inputs()[2].name

## Handlers and control functions ##

def destroy_target(index, toPublish=False):
    '''
    Make the target unavailable if found
    '''
    global target_status
    target_status[index] = 0
    position_goal[0][index][0:3] = [float('inf'), float('inf'), float('inf')]
    if toPublish:
        pub.publish(target_status)

def destroy_target_ros(msg):
    '''
    ROS handler for destroy_target(index, toPublish)
    '''
    global target_status
    target_status = msg.data

    for i in len(target_status):
        if not target_status[i]:
            position_goal[0][i][0:3] = [float('inf'), float('inf'), float('inf')]

def update_position(msg, position: np.array, relative: bool):
    '''
    Update position variable (type: pose.position)
    through ROS & vrpn_client_ros
    '''
    value = msg.pose.position
    position[:3] = [value.x, value.z, value.y]
    if relative:
        position[:3] -= position_drone

def update_orientation(msg, orientation: np.array):
    '''
    Update orientation variable (type: pose.orientation)
    through ROS & vrpn_client_ros
    '''
    value = msg.pose.orientation
    orientation[:3] = [value.x, value.y, value.z, value.w]

def update_all(msg, position: np.array, orientation: np.array):
    '''
    Update position and orientation variables simultaneously
    through ROS & vrpn_client_ros
    '''
    update_orientation(msg, position)
    update_orientation(msg, orientation)

    
def scale_vel_cmd(cmd_val, scale=0.3):
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

def handler(event, sender, data, **args):
    '''
    TelloPy handler function
    '''
    drone = sender
    if event is drone.EVENT_FLIGHT_DATA:
        pass
        
def main():
    global obs_0_data, obs_2_data
    global position_drone, orientation_drone, position_goal
    global frame_count

    drone = tellopy.Tello()

    try:
        drone.subscribe(drone.EVENT_FLIGHT_DATA, handler)

        drone.connect()
        drone.wait_for_connection(60.0)

        drone.takeoff()

        time.sleep(5)

        retry = 3
        container = None
        while container is None and 0 < retry:
            retry -= 1
            try:
                container = av.open(drone.get_video_stream())
            except av.AVError as ave:
                print(ave)
                print('retry...')
                pass

        # skip first 300 frames
        frame_skip = 300
        while True:
            for frame in container.decode(video=0):
                if 0 < frame_skip:
                    frame_skip = frame_skip - 1
                    continue
                start_time = time.time()

                ####################

                ##
                if frame_count == 0:
                    start_time_all = time.time()

                frame_count += 1
                ##

                image = np.array(frame.to_image())[:, 120: 839, :]
                processed_frame = np.array(image_processing(image), dtype=np.float32)

                cv2.imshow('Original', cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
                cv2.waitKey(1)

                _, _, continuous_action, _ = sess.run(None, {
                    obs_0: obs_0_data,
                    obs_1: processed_frame,
                    obs_2: obs_2_data,
                })

                mul = np.matmul(
                    quarternion_to_rotation_matrix(orientation_drone[0]),
                    UNIT_VECTOR_X
                )
                normalized_mul = mul / np.linalg.norm(mul)

                obs_2_data[0, 0:3] = position_drone
                obs_2_data[0, 3:7] = orientation_drone
                obs_2_data[0, 7:10] = np.reshape(normalized_mul, (1, 3))
                obs_2_data[0, 10:14] = continuous_action

                cb_cmd_vel(drone, continuous_action)

                distance_targets = np.linalg.norm(position_goal - position_drone, axis=-1)

                if distance_targets.min() < 0.5:
                    destroy_target(distance_targets.argmin(), True)
                    ##
                    print("End Frame:", frame_count)
                    print("End Time:", time.time() - start_time_all)
                    ##
                    cb_cmd_vel(drone, [[0, 0, 0, 0]])
                    drone.land()
                    time.sleep(5)
                    drone.quit()
                    cv2.destroyAllWindows()

                ####################

                if frame.time_base < 1.0/60:
                    time_base = 1.0/60
                else:
                    time_base = frame.time_base
                frame_skip = int((time.time() - start_time)/time_base)

    except Exception as ex:
        exc_type, exc_value, exc_traceback = sys.exc_info()
        traceback.print_exception(exc_type, exc_value, exc_traceback)
        print(ex)
    finally:
        drone.quit()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('attention_based_controller', anonymous=True)

    pub = rospy.Publisher('target_status', Int32MultiArray, queue_size=10)
    rospy.Subscriber("target_status", Int32MultiArray, destroy_target_ros)

    rospy.Subscriber(f'/vrpn_client_node/{telloIDs[0]}/pose', PoseStamped, 
                     partial(update_all, position=position_drone[0], orientation=orientation_drone[0]))
    rospy.Subscriber(f'/vrpn_client_node/{telloIDs[1]}/pose', PoseStamped,
                     partial(update_position, position=obs_2_data[0][0], relative=True))
    # rospy.Subscriber(f'/vrpn_client_node/{telloIDs[2]}/pose', PoseStamped, 
                    #  partial(update_position, position=obs_2_data[0][1], relative=True))

    rospy.Subscriber(f'/vrpn_client_node/{goalIDs[0]}/pose', PoseStamped, 
                     partial(update_position, position=position_goal[0][1]))
    rospy.Subscriber(f'/vrpn_client_node/{goalIDs[1]}/pose', PoseStamped, 
                     partial(update_position, position=position_goal[0][1]))
    # rospy.Subscriber(f'/vrpn_client_node/{goalIDs[2]}/pose', PoseStamped, 
    #                  partial(update_position, position=position_goal[0]))
    
    main()
