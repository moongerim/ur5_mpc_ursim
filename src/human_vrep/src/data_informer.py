#!/usr/bin/env python
import rospy
import time
import pandas as pd
import csv
import numpy as np
from std_msgs.msg import Float64MultiArray, Int32

pos = []
# 2810
for i in range(7148):
    # temp = '/home/robot/workspaces/ur5_mpc_ursim/data_1701/data_'+str(i+1)+'.csv'
    temp=np.loadtxt('/home/robot/workspaces/ur5_mpc_ursim/data_1701/new_data_'+str(i+151)+'.csv', skiprows = 2,delimiter=',')
    # temp = pd.read_csv(temp, quoting=csv.QUOTE_NONNUMERIC)
    # temp = temp.to_numpy()
    l = len(temp)
    if l>100:
        pos.append(temp)

human_spheres = rospy.Publisher('/dataset', Float64MultiArray, queue_size=1)

print("start")
def main():
    global pos
    rospy.init_node('dataset_info', anonymous=True)
    msg = rospy.wait_for_message("/HighController/start", Int32)
    # msg = True
    if(msg):
        for k in range (len(pos)):
            print(k+151)
            temp = pos[k]
            # print(temp[0])
            for i in range (len(temp)):
                point_array = [0]*80
                for a in range(80):
                    point_array[a]=temp[i][a]
                obstacle_data = Float64MultiArray()
                obstacle_data.data = point_array
                human_spheres.publish(obstacle_data)
                time.sleep(5)
            point_array = [0]*80
            for a in range(80):
                point_array[a]=1000
            obstacle_data = Float64MultiArray()
            obstacle_data.data = point_array
            human_spheres.publish(obstacle_data)
            time.sleep(5)
        rospy.spin()
        print("the end")
    print("exit")    

if __name__ == '__main__': main()
