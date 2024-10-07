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
from std_msgs.msg import Int32MultiArray, MultiArrayDimension, MultiArrayLayout

from functools import partial

from poma_helper import *
from ros_helper import *
from tellopy_helper import *

#------------------------------------------------------------------------------------
#--- Experimental Variables: CAN MODIFY ---------------------------------------------

model = "./VisualDrone(POMA).onnx"

telloIDs = ["tello1", "tello2", "tello3"]
goalIDs = ["goal1", "goal2", "goal3"]

num_drone = len(telloIDs)
num_goal = len(goalIDs)

distance_threshold = 0.50

target_status = [1 if i < num_goal else 0 for i in range(5)]

#------------------------------------------------------------------------------------
#--- Global Variables: DO NOT MODIFY ------------------------------------------------

position_drone = np.zeros((1, 3), dtype=np.float32)
orientation_drone = np.zeros((1, 4), dtype=np.float32)

position_goal = np.zeros((1, 5, 3), dtype=np.float32)

obs_0_data = np.zeros((1, 5, 3), dtype=np.float32)
obs_2_data = np.zeros((1, 14), dtype=np.float32)

#------------------------------------------------------------------------------------
#--- Parameter Config: DO NOT MODIFY ------------------------------------------------

frame_count = 0

for i in range(len(target_status)):
    if not target_status[i]:
        position_goal[0][i][0:3] = [float('inf'), float('inf'), float('inf')]

sess = ort.InferenceSession(model, providers=["CPUExecutionProvider"])

obs_0 = sess.get_inputs()[0].name
obs_1 = sess.get_inputs()[1].name
obs_2 = sess.get_inputs()[2].name

#------------------------------------------------------------------------------------

def update_target_status(data):
    '''
    Update target status information when received from another drone
    '''
    global target_status
    target_status = list(data.data)

def ego_obs_update(drone, continuous_action):
    '''
    Update ego observation variable
    '''
    global position_drone, orientation_drone

    mul = np.matmul(
        quarternion_to_rotation_matrix(orientation_drone[0]),
        UNIT_VECTOR_X
    )

    normalized_mul = mul / np.linalg.norm(mul)

    obs_2_data[0, 0:3] = position_drone
    obs_2_data[0, 3:7] = orientation_drone
    obs_2_data[0, 7:10] = np.reshape(normalized_mul, (1, 3))
    obs_2_data[0, 10:14] = continuous_action

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
        print("[INPUT] Press ENTER to start")
        start = input()

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

                if frame_count == 0:    start_time_all = time.time()
                frame_count += 1

                image = np.array(frame.to_image())[:, 120: 839, :]
                processed_frame = np.array(image_processing(image), dtype=np.float32)

                cv2.imshow('Original', cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
                cv2.waitKey(1)

                data = {obs_0: obs_0_data, obs_1: processed_frame, obs_2: obs_2_data}
                _, _, continuous_action, _ = sess.run(None, data)

                ego_obs_update(drone, continuous_action)
                cb_cmd_vel(drone, continuous_action)

                distance_targets = np.linalg.norm(position_goal - position_drone, axis=-1)

                for i in range(5):
                    if target_status[i] == 0:   distance_targets[0][i] = float('inf')

                if distance_targets.min() < distance_threshold:
                    print_flight_performance(frame_count, start_time_all)
                    drone_land(drone)

                ####################

                if frame.time_base < 1.0/60:    time_base = 1.0/60
                else:                           time_base = frame.time_base
                frame_skip = int((time.time() - start_time)/time_base)

    except Exception as ex:
        exc_type, exc_value, exc_traceback = sys.exc_info()
        traceback.print_exception(exc_type, exc_value, exc_traceback)
        print(ex)
    finally:
        drone.quit()
        cv2.destroyAllWindows()

if __name__ == '__main__':

    rospy.init_node('drone_status', anonymous=True)

    rospy.Subscriber('/target_status_channel', Int32MultiArray, update_target_status)

    rospy.Subscriber(f'/vrpn_client_node/{telloIDs[0]}/pose', PoseStamped, 
                     partial(update_all, position=position_drone[0], orientation=orientation_drone[0]))

    for i in range(1, num_drone):
        rospy.Subscriber(f'/vrpn_client_node/{telloIDs[i]}/pose', PoseStamped,
                     partial(update_position, position=obs_0_data[0][i - 1], relative=True, position_drone=position_drone))

    for i in range(num_goal):
        rospy.Subscriber(f'/vrpn_client_node/{goalIDs[i]}/pose', PoseStamped, 
                     partial(update_position, position=position_goal[0][i], toUpdate=target_status[i]))
    
    main()
