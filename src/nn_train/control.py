#!/usr/bin/env python
import rospy
import numpy as np
import time
import pandas as pd
import os
import csv
from std_msgs.msg import Float64MultiArray, Int32
import math
from sensor_msgs.msg import JointState

pub = rospy.Publisher('/info', Float64MultiArray, queue_size=1)
position = [0]*6
velocity = [0]*6
human_data = [2.5]*42

def callback(data):
    global position, velocity
    position = data.position[0:6]
    velocity = data.velocity[0:6]

def h_callback(data):
    global human_data
    human_data = data.data[0:42]

def main():
    global position, velocity, human_data
    rospy.init_node('control', anonymous=True)
    rospy.Subscriber("/joint_states", JointState, callback)
    rospy.Subscriber("/human", Float64MultiArray, h_callback)
    # sleep_time = 0
    rate = rospy.Rate(120)
    while not rospy.is_shutdown():
        point_array = [0]*54
        point_array[0:6] = position[0:6]
        point_array[6:48] = human_data[0:42]
        point_array[48:54] = velocity[0:6]
        infodata = Float64MultiArray()
        infodata.data = point_array
        rospy.loginfo(infodata)
        pub.publish(infodata)
        rate.sleep()
        # rospy.spin() 

if __name__ == '__main__': main()
