#!/usr/bin/env python
import __main__ as main
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import time
from math import sin,cos,sqrt
from initialization import set_init_pose
from std_msgs.msg import String
import stream_tee as stream_tee
from stream_tee import write_mat
start_time = time. time()

        
class ENV:
    def __init__(self,run_name):
        rospy.Subscriber('/joint_states', JointState, self.callback)
        self.pub = rospy.Publisher('/ur_driver/URScript', String, queue_size=1)
        self.pub_2 = rospy.Publisher('/given_jv', Float64MultiArray,queue_size=1)
        self.obs = [0]*12
        self.run_name = run_name
        self.iter_n = 0
        self.init_log_variables()

    def callback(self, data):
        self.obs[0:6] = data.position[0:6]
        self.obs[6:12] = data.velocity[0:6]

    def step(self,vel,joint):
        hello_str = "speedj(["+str(vel[0])+","+str(vel[1])+","+str(vel[2])+","+str(vel[3])+","+str(vel[4])+","+str(vel[5])+"],"+"5.0"+",0.1)" 
        # rospy.loginfo(hello_str)
        self.pub.publish(hello_str)
        # temp_data = [vel[0], vel[1], vel[2], vel[3], vel[4], vel[5], joint]
        # vel_values = Float64MultiArray()
        # vel_values.data = temp_data
        # self.pub_2.publish(vel_values)
        joint_states = self.observation()
        # print("temp:",joint_states[6:12])
        
        # print("temp:", temp)
        self.cposes(joint_states[0:6])
        real_vels = self.tp_vels(joint_states[0:6],joint_states[6:12])
        given_vels = self.tp_vels(joint_states[0:6],vel[0:6])
        lin_v_real=[0]*10
        lin_v_given=[0]*10
        for k in range(10):
            lin_v_real[k] = sqrt(real_vels[k*3]*real_vels[k*3]+real_vels[k*3+1]*real_vels[k*3+1]+real_vels[k*3+2]*real_vels[k*3+2])
        
        for k in range(10):
            lin_v_given[k] = sqrt(given_vels[k*3]*given_vels[k*3]+given_vels[k*3+1]*given_vels[k*3+1]+given_vels[k*3+2]*given_vels[k*3+2])
    
        end_time = time.time()
        time_elapsed = (end_time - start_time)


        # self.state.append(joint_states)
        self.poses.append(joint_states[0:6])
        self.joint_velocities.append(joint_states[6:12])
        # print("js:", joint_states)
        self.real_lin_vel.append(lin_v_real)
        self.given_lin_vel.append(lin_v_given)
        self.tp_table.append(self.tp)
        self.real_vel.append(vel)
        self.time_t.append(time_elapsed)
        self.joint_n.append(joint)

    def cposes(self,theta=None):
        if theta is None:
            theta=self.observation[0:6]
        theta_1 = theta[0]
        theta_2 = theta[1]
        theta_3 = theta[2]
        theta_4 = theta[3]
        theta_5 = theta[4]
        theta_6 = theta[5]
        
        self.tp[0] = 1*(0.4*sin(theta_1)-0.425*cos(theta_1)*cos(theta_2))/3
        self.tp[1] = 1*(-0.4*cos(theta_1)-0.425*cos(theta_2)*sin(theta_1))/3
        self.tp[2] = 0.08945+(0.08945-0.425*sin(theta_2)-0.08945)/3
        self.tp[3] = 2*(0.2*sin(theta_1) - 0.425*cos(theta_1)*cos(theta_2))/3
        self.tp[4] = 2*(-0.2*cos(theta_1) - 0.425*cos(theta_2)*sin(theta_1))/3
        self.tp[5] = 0.08945+2*(0.08945-0.425*sin(theta_2)-0.08945)/3
        self.tp[6] = 0.11*sin(theta_1) - 0.425*cos(theta_1)*cos(theta_2)
        self.tp[7] = -0.11*cos(theta_1) - 0.425*cos(theta_2)*sin(theta_1)
        self.tp[8] = 0.08945  -0.425*sin(theta_2)
        self.tp[9] = -0.425*cos(theta_1)*cos(theta_2)+(-(cos(theta_1)*(1569.0*cos(theta_2 + theta_3) + 1700.0*cos(theta_2)))/4000-(-0.425*cos(theta_1)*cos(theta_2)))/4
        self.tp[10] = -0.425*cos(theta_2)*sin(theta_1)+(-(sin(theta_1)*(1569.0*cos(theta_2 + theta_3) + 1700.0*cos(theta_2)))/4000-(-0.425*cos(theta_2)*sin(theta_1)))/4
        self.tp[11] = 0.08945 - 0.425*sin(theta_2)+(0.08945 - 0.425*sin(theta_2) - 0.39225*sin(theta_2 + theta_3)-(0.08945 - 0.425*sin(theta_2)))/4
        self.tp[12] = -0.425*cos(theta_1)*cos(theta_2)+2*(-(cos(theta_1)*(1569.0*cos(theta_2 + theta_3) + 1700.0*cos(theta_2)))/4000-(-0.425*cos(theta_1)*cos(theta_2)))/4
        self.tp[13] = -0.425*cos(theta_2)*sin(theta_1)+2*(-(sin(theta_1)*(1569.0*cos(theta_2 + theta_3) + 1700.0*cos(theta_2)))/4000-(-0.425*cos(theta_2)*sin(theta_1)))/4
        self.tp[14] = 0.08945 - 0.425*sin(theta_2)+2*(0.08945 - 0.425*sin(theta_2) - 0.39225*sin(theta_2 + theta_3)-(0.08945 - 0.425*sin(theta_2)))/4
        self.tp[15] = -0.425*cos(theta_1)*cos(theta_2)+3*(-(cos(theta_1)*(1569.0*cos(theta_2 + theta_3) + 1700.0*cos(theta_2)))/4000-(-0.425*cos(theta_1)*cos(theta_2)))/4
        self.tp[16] = -0.425*cos(theta_2)*sin(theta_1)+3*(-(sin(theta_1)*(1569.0*cos(theta_2 + theta_3) + 1700.0*cos(theta_2)))/4000-(-0.425*cos(theta_2)*sin(theta_1)))/4
        self.tp[17] = 0.08945 - 0.425*sin(theta_2)+3*(0.08945 - 0.425*sin(theta_2) - 0.39225*sin(theta_2 + theta_3)-(0.08945 - 0.425*sin(theta_2)))/4
        self.tp[18] = -(cos(theta_1)*(1569.0*cos(theta_2 + theta_3) + 1700.0*cos(theta_2)))/4000
        self.tp[19] = -(sin(theta_1)*(1569.0*cos(theta_2 + theta_3) + 1700.0*cos(theta_2)))/4000
        self.tp[20] = 0.08945 - 0.425*sin(theta_2) - 0.39225*sin(theta_2 + theta_3)
        self.tp[21] = 0.10915*sin(theta_1) - 0.425*cos(theta_1)*cos(theta_2) + 0.39225*cos(theta_1)*sin(theta_2)*sin(theta_3) - 0.39225*cos(theta_1)*cos(theta_2)*cos(theta_3)
        self.tp[22] = 0.39225*sin(theta_1)*sin(theta_2)*sin(theta_3) - 0.425*cos(theta_2)*sin(theta_1) - 0.10915*cos(theta_1) - 0.39225*cos(theta_2)*cos(theta_3)*sin(theta_1)
        self.tp[23] = 0.08945 - 0.425*sin(theta_2) - 0.39225*sin(theta_2 + theta_3)
        self.tp[24] = 0.10915*sin(theta_1) - 0.425*cos(theta_1)*cos(theta_2) + 0.09465*cos(theta_4)*(cos(theta_1)*cos(theta_2)*sin(theta_3) + cos(theta_1)*cos(theta_3)*sin(theta_2)) - 0.09465*sin(theta_4)*(cos(theta_1)*sin(theta_2)*sin(theta_3) - 1.0*cos(theta_1)*cos(theta_2)*cos(theta_3)) + 0.39225*cos(theta_1)*sin(theta_2)*sin(theta_3) - 0.39225*cos(theta_1)*cos(theta_2)*cos(theta_3)
        self.tp[25] = 0.09465*cos(theta_4)*(cos(theta_2)*sin(theta_1)*sin(theta_3) + cos(theta_3)*sin(theta_1)*sin(theta_2)) - 0.425*cos(theta_2)*sin(theta_1) - 0.10915*cos(theta_1) - 0.09465*sin(theta_4)*(sin(theta_1)*sin(theta_2)*sin(theta_3) - 1.0*cos(theta_2)*cos(theta_3)*sin(theta_1)) + 0.39225*sin(theta_1)*sin(theta_2)*sin(theta_3) - 0.39225*cos(theta_2)*cos(theta_3)*sin(theta_1)
        self.tp[26] = 0.08945 - 0.39225*sin(theta_2 + theta_3) - 0.425*sin(theta_2) - 0.09465*cos(theta_2 + theta_3 + theta_4)
        self.tp[27] = 0.10915*sin(theta_1) - 0.425*cos(theta_1)*cos(theta_2) + 0.0823*cos(theta_5)*sin(theta_1) + 0.39225*cos(theta_1)*sin(theta_2)*sin(theta_3) - 0.0823*cos(theta_2 + theta_3 + theta_4)*cos(theta_1)*sin(theta_5) + 0.09465*cos(theta_2 + theta_3)*cos(theta_1)*sin(theta_4) + 0.09465*sin(theta_2 + theta_3)*cos(theta_1)*cos(theta_4) - 0.39225*cos(theta_1)*cos(theta_2)*cos(theta_3)
        self.tp[28] = 0.39225*sin(theta_1)*sin(theta_2)*sin(theta_3) - 0.0823*cos(theta_1)*cos(theta_5) - 0.425*cos(theta_2)*sin(theta_1) - 0.10915*cos(theta_1) - 0.0823*cos(theta_2 + theta_3 + theta_4)*sin(theta_1)*sin(theta_5) + 0.09465*cos(theta_2 + theta_3)*sin(theta_1)*sin(theta_4) + 0.09465*sin(theta_2 + theta_3)*cos(theta_4)*sin(theta_1) - 0.39225*cos(theta_2)*cos(theta_3)*sin(theta_1)
        self.tp[29] = 0.09465*sin(theta_2 + theta_3)*sin(theta_4) - 0.425*sin(theta_2) - 0.39225*sin(theta_2 + theta_3) - sin(theta_5)*(0.0823*cos(theta_2 + theta_3)*sin(theta_4) + 0.0823*sin(theta_2 + theta_3)*cos(theta_4)) - 0.09465*cos(theta_2 + theta_3)*cos(theta_4) + 0.08945
    
    def tp_vels(self,theta,u):
        theta_1 = theta[0]
        theta_2 = theta[1]
        theta_3 = theta[2]
        theta_4 = theta[3]
        theta_5 = theta[4]
        theta_6 = theta[5]
        u_1 = u[0]
        u_2 = u[1]
        u_3 = u[2]    
        u_4 = u[3]
        u_5 = u[4]
        u_6 = u[5]
        tp_v = [0 for c in range(30)]
        tp_v[0] = (0.425*u_1*cos(theta_2)*sin(theta_1) + 0.425*u_2*cos(theta_1)*sin(theta_2))/3
        tp_v[1] = (0.425*u_2*sin(theta_1)*sin(theta_2) - 0.425*u_1*cos(theta_1)*cos(theta_2))/3
        tp_v[2] = (-0.425*u_2*cos(theta_2))/3
        tp_v[3] = 2*(0.425*u_1*cos(theta_2)*sin(theta_1) + 0.425*u_2*cos(theta_1)*sin(theta_2))/3
        tp_v[4] = 2*(0.425*u_2*sin(theta_1)*sin(theta_2) - 0.425*u_1*cos(theta_1)*cos(theta_2))/3
        tp_v[5] = 2*(-0.425*u_2*cos(theta_2))/3
        tp_v[6] = 0.425*u_1*cos(theta_2)*sin(theta_1) + 0.425*u_2*cos(theta_1)*sin(theta_2)
        tp_v[7] = 0.425*u_2*sin(theta_1)*sin(theta_2) - 0.425*u_1*cos(theta_1)*cos(theta_2)
        tp_v[8] = -0.425*u_2*cos(theta_2)
        tp_v[9] = 0.425*u_1*cos(theta_2)*sin(theta_1) + 0.425*u_2*cos(theta_1)*sin(theta_2)+1/4*(u_2*(0.425*cos(theta_1)*sin(theta_2) + 0.39225*cos(theta_1)*cos(theta_2)*sin(theta_3) + 0.39225*cos(theta_1)*cos(theta_3)*sin(theta_2)) + u_1*(0.425*cos(theta_2)*sin(theta_1) - 0.39225*sin(theta_1)*sin(theta_2)*sin(theta_3) + 0.39225*cos(theta_2)*cos(theta_3)*sin(theta_1)) + u_3*(0.39225*cos(theta_1)*cos(theta_2)*sin(theta_3) + 0.39225*cos(theta_1)*cos(theta_3)*sin(theta_2))-(0.425*u_1*cos(theta_2)*sin(theta_1) + 0.425*u_2*cos(theta_1)*sin(theta_2)))
        tp_v[10] = 0.425*u_2*sin(theta_1)*sin(theta_2) - 0.425*u_1*cos(theta_1)*cos(theta_2)+1/4*(u_3*(0.39225*cos(theta_2)*sin(theta_1)*sin(theta_3) + 0.39225*cos(theta_3)*sin(theta_1)*sin(theta_2)) - 1.0*u_1*(0.425*cos(theta_1)*cos(theta_2) - 0.39225*cos(theta_1)*sin(theta_2)*sin(theta_3) + 0.39225*cos(theta_1)*cos(theta_2)*cos(theta_3)) + u_2*(0.425*sin(theta_1)*sin(theta_2) + 0.39225*cos(theta_2)*sin(theta_1)*sin(theta_3) + 0.39225*cos(theta_3)*sin(theta_1)*sin(theta_2))-(0.425*u_2*sin(theta_1)*sin(theta_2) - 0.425*u_1*cos(theta_1)*cos(theta_2)))
        tp_v[11] = -0.425*u_2*cos(theta_2)+1/4*(- 1.0*u_2*(0.39225*cos(theta_2 + theta_3) + 0.425*cos(theta_2)) - 0.39225*u_3*cos(theta_2 + theta_3)-(-0.425*u_2*cos(theta_2)))
        tp_v[12] = 0.425*u_1*cos(theta_2)*sin(theta_1) + 0.425*u_2*cos(theta_1)*sin(theta_2)+2/4*(u_2*(0.425*cos(theta_1)*sin(theta_2) + 0.39225*cos(theta_1)*cos(theta_2)*sin(theta_3) + 0.39225*cos(theta_1)*cos(theta_3)*sin(theta_2)) + u_1*(0.425*cos(theta_2)*sin(theta_1) - 0.39225*sin(theta_1)*sin(theta_2)*sin(theta_3) + 0.39225*cos(theta_2)*cos(theta_3)*sin(theta_1)) + u_3*(0.39225*cos(theta_1)*cos(theta_2)*sin(theta_3) + 0.39225*cos(theta_1)*cos(theta_3)*sin(theta_2))-(0.425*u_1*cos(theta_2)*sin(theta_1) + 0.425*u_2*cos(theta_1)*sin(theta_2)))
        tp_v[13] = 0.425*u_2*sin(theta_1)*sin(theta_2) - 0.425*u_1*cos(theta_1)*cos(theta_2)+2/4*(u_3*(0.39225*cos(theta_2)*sin(theta_1)*sin(theta_3) + 0.39225*cos(theta_3)*sin(theta_1)*sin(theta_2)) - 1.0*u_1*(0.425*cos(theta_1)*cos(theta_2) - 0.39225*cos(theta_1)*sin(theta_2)*sin(theta_3) + 0.39225*cos(theta_1)*cos(theta_2)*cos(theta_3)) + u_2*(0.425*sin(theta_1)*sin(theta_2) + 0.39225*cos(theta_2)*sin(theta_1)*sin(theta_3) + 0.39225*cos(theta_3)*sin(theta_1)*sin(theta_2))-(0.425*u_2*sin(theta_1)*sin(theta_2) - 0.425*u_1*cos(theta_1)*cos(theta_2)))
        tp_v[14] = -0.425*u_2*cos(theta_2)+2/4*(- 1.0*u_2*(0.39225*cos(theta_2 + theta_3) + 0.425*cos(theta_2)) - 0.39225*u_3*cos(theta_2 + theta_3)-(-0.425*u_2*cos(theta_2)))
        tp_v[15] = 0.425*u_1*cos(theta_2)*sin(theta_1) + 0.425*u_2*cos(theta_1)*sin(theta_2)+3/4*(u_2*(0.425*cos(theta_1)*sin(theta_2) + 0.39225*cos(theta_1)*cos(theta_2)*sin(theta_3) + 0.39225*cos(theta_1)*cos(theta_3)*sin(theta_2)) + u_1*(0.425*cos(theta_2)*sin(theta_1) - 0.39225*sin(theta_1)*sin(theta_2)*sin(theta_3) + 0.39225*cos(theta_2)*cos(theta_3)*sin(theta_1)) + u_3*(0.39225*cos(theta_1)*cos(theta_2)*sin(theta_3) + 0.39225*cos(theta_1)*cos(theta_3)*sin(theta_2))-(0.425*u_1*cos(theta_2)*sin(theta_1) + 0.425*u_2*cos(theta_1)*sin(theta_2)))
        tp_v[16] = 0.425*u_2*sin(theta_1)*sin(theta_2) - 0.425*u_1*cos(theta_1)*cos(theta_2)+3/4*(u_3*(0.39225*cos(theta_2)*sin(theta_1)*sin(theta_3) + 0.39225*cos(theta_3)*sin(theta_1)*sin(theta_2)) - 1.0*u_1*(0.425*cos(theta_1)*cos(theta_2) - 0.39225*cos(theta_1)*sin(theta_2)*sin(theta_3) + 0.39225*cos(theta_1)*cos(theta_2)*cos(theta_3)) + u_2*(0.425*sin(theta_1)*sin(theta_2) + 0.39225*cos(theta_2)*sin(theta_1)*sin(theta_3) + 0.39225*cos(theta_3)*sin(theta_1)*sin(theta_2))-(0.425*u_2*sin(theta_1)*sin(theta_2) - 0.425*u_1*cos(theta_1)*cos(theta_2)))
        tp_v[17] = -0.425*u_2*cos(theta_2)+3/4*(- 1.0*u_2*(0.39225*cos(theta_2 + theta_3) + 0.425*cos(theta_2)) - 0.39225*u_3*cos(theta_2 + theta_3)-(-0.425*u_2*cos(theta_2)))
        tp_v[18] = u_2*(0.425*cos(theta_1)*sin(theta_2) + 0.39225*cos(theta_1)*cos(theta_2)*sin(theta_3) + 0.39225*cos(theta_1)*cos(theta_3)*sin(theta_2)) + u_1*(0.425*cos(theta_2)*sin(theta_1) - 0.39225*sin(theta_1)*sin(theta_2)*sin(theta_3) + 0.39225*cos(theta_2)*cos(theta_3)*sin(theta_1)) + u_3*(0.39225*cos(theta_1)*cos(theta_2)*sin(theta_3) + 0.39225*cos(theta_1)*cos(theta_3)*sin(theta_2))
        tp_v[19] = u_3*(0.39225*cos(theta_2)*sin(theta_1)*sin(theta_3) + 0.39225*cos(theta_3)*sin(theta_1)*sin(theta_2)) - 1.0*u_1*(0.425*cos(theta_1)*cos(theta_2) - 0.39225*cos(theta_1)*sin(theta_2)*sin(theta_3) + 0.39225*cos(theta_1)*cos(theta_2)*cos(theta_3)) + u_2*(0.425*sin(theta_1)*sin(theta_2) + 0.39225*cos(theta_2)*sin(theta_1)*sin(theta_3) + 0.39225*cos(theta_3)*sin(theta_1)*sin(theta_2))
        tp_v[20] = - 1.0*u_2*(0.39225*cos(theta_2 + theta_3) + 0.425*cos(theta_2)) - 0.39225*u_3*cos(theta_2 + theta_3)
        tp_v[21] = u_1*(0.10915*cos(theta_1) + 0.425*cos(theta_2)*sin(theta_1) - 0.39225*sin(theta_1)*sin(theta_2)*sin(theta_3) + 0.39225*cos(theta_2)*cos(theta_3)*sin(theta_1)) + 0.00025*u_2*cos(theta_1)*(1569.0*sin(theta_2 + theta_3) + 1700.0*sin(theta_2)) + 0.39225*u_3*sin(theta_2 + theta_3)*cos(theta_1)
        tp_v[22] = u_1*(0.10915*sin(theta_1) - 0.425*cos(theta_1)*cos(theta_2) + 0.39225*cos(theta_1)*sin(theta_2)*sin(theta_3) - 0.39225*cos(theta_1)*cos(theta_2)*cos(theta_3)) + 0.00025*u_2*sin(theta_1)*(1569.0*sin(theta_2 + theta_3) + 1700.0*sin(theta_2)) + 0.39225*u_3*sin(theta_2 + theta_3)*sin(theta_1)
        tp_v[23] = - 1.0*u_2*(0.39225*cos(theta_2 + theta_3) + 0.425*cos(theta_2)) - 0.39225*u_3*cos(theta_2 + theta_3)
        tp_v[24] = u_1*(0.10915*cos(theta_1) + 0.425*cos(theta_2)*sin(theta_1) - 0.39225*sin(theta_1)*sin(theta_2)*sin(theta_3) - 0.09465*cos(theta_2 + theta_3)*sin(theta_1)*sin(theta_4) - 0.09465*sin(theta_2 + theta_3)*cos(theta_4)*sin(theta_1) + 0.39225*cos(theta_2)*cos(theta_3)*sin(theta_1)) + 0.00005*u_2*cos(theta_1)*(1893.0*cos(theta_2 + theta_3 + theta_4) + 7845.0*sin(theta_2 + theta_3) + 8500.0*sin(theta_2)) + 0.00015*u_3*cos(theta_1)*(631.0*cos(theta_2 + theta_3 + theta_4) + 2615.0*sin(theta_2 + theta_3)) + 0.09465*u_4*cos(theta_2 + theta_3 + theta_4)*cos(theta_1)
        tp_v[25] = u_1*(0.10915*sin(theta_1) - 0.425*cos(theta_1)*cos(theta_2) + 0.39225*cos(theta_1)*sin(theta_2)*sin(theta_3) + 0.09465*cos(theta_2 + theta_3)*cos(theta_1)*sin(theta_4) + 0.09465*sin(theta_2 + theta_3)*cos(theta_1)*cos(theta_4) - 0.39225*cos(theta_1)*cos(theta_2)*cos(theta_3)) + 0.00005*u_2*sin(theta_1)*(1893.0*cos(theta_2 + theta_3 + theta_4) + 7845.0*sin(theta_2 + theta_3) + 8500.0*sin(theta_2)) + 0.00015*u_3*sin(theta_1)*(631.0*cos(theta_2 + theta_3 + theta_4) + 2615.0*sin(theta_2 + theta_3)) + 0.09465*u_4*cos(theta_2 + theta_3 + theta_4)*sin(theta_1)
        tp_v[26] = 0.09465*u_2*sin(theta_2 + theta_3 + theta_4) + 0.09465*u_3*sin(theta_2 + theta_3 + theta_4) + 0.09465*u_4*sin(theta_2 + theta_3 + theta_4) - 0.39225*u_2*cos(theta_2 + theta_3) - 0.39225*u_3*cos(theta_2 + theta_3) - 0.425*u_2*cos(theta_2)
        tp_v[27] = u_1*(0.10915*cos(theta_1) + 0.0823*cos(theta_1)*cos(theta_5) + 0.425*cos(theta_2)*sin(theta_1) - 0.39225*sin(theta_1)*sin(theta_2)*sin(theta_3) + 0.0823*cos(theta_2 + theta_3 + theta_4)*sin(theta_1)*sin(theta_5) - 0.09465*cos(theta_2 + theta_3)*sin(theta_1)*sin(theta_4) - 0.09465*sin(theta_2 + theta_3)*cos(theta_4)*sin(theta_1) + 0.39225*cos(theta_2)*cos(theta_3)*sin(theta_1)) - 1.0*u_5*(0.0823*sin(theta_1)*sin(theta_5) + 0.0823*cos(theta_2 + theta_3 + theta_4)*cos(theta_1)*cos(theta_5)) + 0.00005*u_2*cos(theta_1)*(1893.0*cos(theta_2 + theta_3 + theta_4) + 823.0*cos(theta_2 + theta_3 + theta_4 - 1.0*theta_5) - 823.0*cos(theta_2 + theta_3 + theta_4 + theta_5) + 7845.0*sin(theta_2 + theta_3) + 8500.0*sin(theta_2)) + 0.00005*u_4*cos(theta_1)*(1646.0*sin(theta_2 + theta_3 + theta_4)*sin(theta_5) - 1893.0*sin(theta_2 + theta_3)*sin(theta_4) + 1893.0*cos(theta_2 + theta_3)*cos(theta_4)) + 0.00005*u_3*cos(theta_1)*(1893.0*cos(theta_2 + theta_3 + theta_4) + 823.0*cos(theta_2 + theta_3 + theta_4 - 1.0*theta_5) - 823.0*cos(theta_2 + theta_3 + theta_4 + theta_5) + 7845.0*sin(theta_2 + theta_3))
        tp_v[28] = u_5*(0.0823*cos(theta_1)*sin(theta_5) - 0.0823*cos(theta_2 + theta_3 + theta_4)*cos(theta_5)*sin(theta_1)) + u_1*(0.10915*sin(theta_1) - 0.425*cos(theta_1)*cos(theta_2) + 0.0823*cos(theta_5)*sin(theta_1) + 0.39225*cos(theta_1)*sin(theta_2)*sin(theta_3) - 0.0823*cos(theta_2 + theta_3 + theta_4)*cos(theta_1)*sin(theta_5) + 0.09465*cos(theta_2 + theta_3)*cos(theta_1)*sin(theta_4) + 0.09465*sin(theta_2 + theta_3)*cos(theta_1)*cos(theta_4) - 0.39225*cos(theta_1)*cos(theta_2)*cos(theta_3)) + 0.00005*u_2*sin(theta_1)*(1893.0*cos(theta_2 + theta_3 + theta_4) + 823.0*cos(theta_2 + theta_3 + theta_4 - 1.0*theta_5) - 823.0*cos(theta_2 + theta_3 + theta_4 + theta_5) + 7845.0*sin(theta_2 + theta_3) + 8500.0*sin(theta_2)) + 0.00005*u_4*sin(theta_1)*(1646.0*sin(theta_2 + theta_3 + theta_4)*sin(theta_5) - 1893.0*sin(theta_2 + theta_3)*sin(theta_4) + 1893.0*cos(theta_2 + theta_3)*cos(theta_4)) + 0.00005*u_3*sin(theta_1)*(1893.0*cos(theta_2 + theta_3 + theta_4) + 823.0*cos(theta_2 + theta_3 + theta_4 - 1.0*theta_5) - 823.0*cos(theta_2 + theta_3 + theta_4 + theta_5) + 7845.0*sin(theta_2 + theta_3))
        tp_v[29] = u_4*(0.09465*sin(theta_2 + theta_3 + theta_4) - 0.0823*cos(theta_2 + theta_3 + theta_4)*sin(theta_5)) - 1.0*u_3*(0.39225*cos(theta_2 + theta_3) - 0.09465*sin(theta_2 + theta_3 + theta_4) + 0.0823*cos(theta_2 + theta_3 + theta_4)*sin(theta_5)) - 1.0*u_2*(0.39225*cos(theta_2 + theta_3) + 0.425*cos(theta_2) - 0.09465*cos(theta_2 + theta_3)*sin(theta_4) - 0.09465*sin(theta_2 + theta_3)*cos(theta_4) + 0.0823*cos(theta_2 + theta_3)*cos(theta_4)*sin(theta_5) - 0.0823*sin(theta_2 + theta_3)*sin(theta_4)*sin(theta_5)) - 0.0823*u_5*sin(theta_2 + theta_3 + theta_4)*cos(theta_5)
        return tp_v

    def observation(self):
        return self.obs

    def reset(self, init):  
        self.save_log()
        self.iter_n+=1 
        set_init_pose(init)
        self.init_log_variables()
        
    def init_log_variables(self):
        self.poses=[]
        self.joint_velocities=[]
        self.real_lin_vel=[]
        self.given_lin_vel=[]
        self.tp_table=[]
        self.real_vel=[]
        self.time_t=[]
        self.joint_n=[]
        self.tp = [0 for c in range(30)]
    
    def save_log(self):
        write_mat('Network_log/' + self.run_name,
                        {'poses': self.poses,
                        'joint_velocities': self.joint_velocities,
                        'real_lin_vel': self.real_lin_vel,
                        'given_lin_vel':self.given_lin_vel,
                        'tp_table': self.tp_table,
                        'real_vel': self.real_vel,
                        'time_t': self.time_t,
                        'joint_n':self.joint_n},
                        str(self.iter_n))

if __name__ == '__main__':
    rospy.init_node("accel_test", anonymous=True)
    run_name = stream_tee.generate_timestamp()
    env = ENV(run_name)
    t = time.time()
    # env.reset()
    i = 0
    iteration = 0
    max_vel = 0.4
    rate = rospy.Rate(200) #hz
    joint = 1
    zero_vel = [0,0,0,0,0,0]
    init = [0.7853, 0, 0, -1.5708, 0, 0]
    while not rospy.is_shutdown():
        if joint == 1:
            vel = [max_vel,0,0,0,0,0]
        if joint == 2:
            vel = [0, -max_vel,0,0,0,0]
        if joint == 3:
            vel = [0, 0, -max_vel,0,0,0]
            init = [0.7853, 0, 1.57, -1.5708, 0, 0]
        if joint == 4:
            vel = [0, 0, 0, -max_vel,0,0]
        if joint == 5:
            vel = [0, 0, 0, 0, max_vel,0]
        if joint == 6:
            vel = [0, 0, 0, 0, 0, max_vel]
        if joint == 7:
            vel = [max_vel,-max_vel,-max_vel,-max_vel,max_vel,max_vel]

        obs = env.observation()

        print(joint, max_vel, i, obs[6:12])
        
        if obs[6]==0.0 and obs[7]==0.0 and obs[8]==0.0 and obs[9]==0.0 and obs[10]==0.0 and obs[11]==0.0:
            start = 1

        if max_vel<1.3:
            if i>600:
                if start==0 and i<1200:
                    env.step(zero_vel, joint)
                    print(i, zero_vel)
                else:
                    env.reset(init)
                    time.sleep(20)
                    max_vel+=0.05
                    start_time = time.time()
                    i = 0
            else:
                env.step(vel, joint)
                print(i,vel)
                start=0
            i+=1
                
        else:
            joint+=1
            i=0
            max_vel = 0.4

        rate.sleep()
    
        