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

telloIDs = ["tello1", "tello2", "tello3"]
goalIDs = ["goal1", "goal2", "goal3"]

distance_threshold = 0.5
#------------------------------------------------------------------------------------
#--- Global Variables: DO NOT MODIFY ------------------------------------------------

num_drone = len(telloIDs)
num_goal = len(goalIDs)

target_status = [1 if i < num_goal else 0 for i in range(5)]

position_drone = np.zeros((num_drone, 3), dtype=np.float32)
position_goal = np.zeros((num_goal, 3), dtype=np.float32)

#------------------------------------------------------------------------------------
#--- Parameter Config: DO NOT MODIFY ------------------------------------------------


dim = MultiArrayDimension()
dim.size = num_goal
dim.stride = 1
dim.label = "my_data"

layout = MultiArrayLayout()
layout.dim.append(dim)
layout.data_offset = 0

message = Int32MultiArray()
message.layout = layout
message.data = target_status

for i in range(num_goal):
    if not target_status[i]:
        position_goal[i][0:3] = [float('inf'), float('inf'), float('inf')]

#------------------------------------------------------------------------------------

def destroy_target(index):
    '''
    Make the target unavailable if found
    '''
    global target_status, position_goal
    
    target_status[index] = 0
    position_goal[index][0:3] = [float('inf'), float('inf'), float('inf')]

def calculate_distance(drone_i, target_j):
    global position_drone, position_goal
    return np.linalg.norm(position_drone[drone_i, :] - position_goal[target_j, :])

vectorized_calculate_distance = np.vectorize(calculate_distance)

def threshold_check(threshold=0.5):
    result = vectorized_calculate_distance(np.arange(num_drone)[:, None], np.arange(num_goal))
    index = np.unravel_index(np.argmin(result), result.shape) # drone_i, target_j
    return result.min() < threshold, index[1]

#------------------------------------------------------------------------------------

def main():
    global position_drone, orientation_drone, position_goal
    global frame_count

    time.sleep(5)

    rate = rospy.Rate(240)

    while not rospy.is_shutdown():
        
        rate.sleep()
        
        check_result, index = threshold_check()

        if check_result:
            destroy_target(index)
        
        message.data = target_status
        pub.publish(message)

if __name__ == '__main__':
    rospy.init_node('drone_status_master', anonymous=True)

    pub = rospy.Publisher('/target_status_channel', Int32MultiArray, queue_size=10)

    for i in range(num_drone):
        rospy.Subscriber(f'/vrpn_client_node/{telloIDs[i]}/pose', PoseStamped,
                     partial(update_position, position=position_drone[i], relative=False, toUpdate=True))

    for i in range(num_goal):
        rospy.Subscriber(f'/vrpn_client_node/{goalIDs[i]}/pose', PoseStamped, 
                     partial(update_position, position=position_goal[i], relative=False, toUpdate=target_status[i]))
    
    main()
