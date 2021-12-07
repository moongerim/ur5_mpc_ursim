#!/usr/bin/env python
import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from math import pi
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String

pub = rospy.Publisher('/ur_driver/URScript', String, queue_size=1)
pub_2 = rospy.Publisher('/given_jv', Float64MultiArray,queue_size=1)
i = 0
def talker(data):
    global i
    if i<100:
        vel = [1,1,1,1,1,1]
    else:
        vel = [0,0,0,0,0,0]
    hello_str = "speedj(["+str(vel[0])+","+str(vel[1])+","+str(vel[2])+","+str(vel[3])+","+str(vel[4])+","+str(vel[5])+"],"+"5.0"+",0.1)" 
    rospy.loginfo(hello_str)
    vel_values = Float64MultiArray()
    vel_values.data = vel
    pub.publish(hello_str)
    pub_2.publish(vel_values)
    i+=1
    

def main():
    rospy.init_node("test_move", anonymous=True)
    # rate = rospy.Rate(20) #hz
    while not rospy.is_shutdown():
        rospy.Subscriber("/joint_states", JointState, talker)
        rospy.spin()


if __name__ == '__main__': main()
