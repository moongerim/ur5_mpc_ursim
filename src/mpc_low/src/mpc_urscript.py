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
from initialization import set_init_pose
JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

client = None
pub = rospy.Publisher('/ur_driver/URScript', String, queue_size=1)
 
def talker(data):
    vel = data.data[0:7]
    # print(vel[6])
    # if vel[6]==0:
    hello_str = "speedj(["+str(vel[0])+","+str(vel[1])+","+str(vel[2])+","+str(vel[3])+","+str(vel[4])+","+str(vel[5])+"],"+"5.0"+",0.1)" 
    rospy.loginfo(hello_str)
    pub.publish(hello_str)
    # else:
    #     set_init_pose(vel[0:6])
        

def main():
    rospy.init_node("test_move", anonymous=True)
    # rate = rospy.Rate(20) #hz
    while not rospy.is_shutdown():
        rospy.Subscriber("/LowController/MPC_solutions", Float64MultiArray, talker)
        rospy.spin()


if __name__ == '__main__': main()
