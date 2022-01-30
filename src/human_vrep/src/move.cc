#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include <math.h>
#include <sensor_msgs/JointState.h>
float point_array_temp[57];
float point_array[57];
double from_high[31] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,10};
float sphere_radi[14]={0.5510,0.6010,0.5010,0.5010,0.5010,0.5010,0.5010,0.5010,0.4510,0.4510,0.4810,0.4810,0.5510,0.6010};

void chatterCallback(const std_msgs::Float64MultiArray msg)
{
  for (int i = 0; i<14; i++){
    point_array_temp[i*4] = msg.data[i*3];
    point_array_temp[i*4+1] = msg.data[i*3+1];
    point_array_temp[i*4+2] = msg.data[i*3+2];
    //vrep diff:
    point_array_temp[i*4+3] = sphere_radi[i];
  }
  point_array_temp[56] = msg.data[42];
}
double state_feedback_temp[12];
double state_feedback[12];
void feedbackCB(const sensor_msgs::JointState msg) 
{
  for (int i = 0; i < 6; ++i) 
  {
    state_feedback_temp[i] = msg.position[i];
    state_feedback_temp[i+6] = msg.velocity[i];
  }
}

void datasetCB(const std_msgs::Float64MultiArray msg) 
{
  for (int i = 0; i < 6; ++i) 
  {
    state_feedback_temp[i] = msg.data[i];
    state_feedback_temp[i+6] = 1;
  }
  for (int i = 0; i<14; i++){
    point_array_temp[i*4] = msg.data[6+i*3];
    point_array_temp[i*4+1] = msg.data[6+i*3+1];
    point_array_temp[i*4+2] = msg.data[6+i*3+2];
    point_array_temp[i*4+3] = sphere_radi[i];
  }
  point_array_temp[56] = msg.data[48];
  for (int i = 0; i < 31; ++i) 
  {
    from_high[i] = msg.data[49+i];
  }
  printf("max diff = %f\n", from_high[30]);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "human_control");
  ros::NodeHandle nodeHandle("~");
  ros::NodeHandle n;
  // ros::Subscriber sub = n.subscribe<std_msgs::Float64MultiArray>("/Obstacle/human_spheres", 1, chatterCallback);
  // ros::Subscriber arm_sub = n.subscribe<sensor_msgs::JointState>("/joint_states", 1, feedbackCB);
  ros::Subscriber arm_sub = n.subscribe<std_msgs::Float64MultiArray>("/dataset", 1, datasetCB);
  ros:: Publisher chatter_low = n.advertise<std_msgs::Float64MultiArray>("/Obstacle/mpc_low_spheres", 1);
  ros:: Publisher chatter_high = n.advertise<std_msgs::Float64MultiArray>("/Obstacle/mpc_high_spheres", 1);
  ros:: Publisher states_low = n.advertise<std_msgs::Float64MultiArray>("/joint_states_low", 1);
  ros:: Publisher states_high = n.advertise<std_msgs::Float64MultiArray>("/joint_states_high", 1);
  // ros:: Publisher chatter_pub = n.advertise<std_msgs::Float64MultiArray>("/HighController/mpc_high_positions", 1);
  // ros::Rate loop_rate(40);
  while (ros::ok())
  {
    for (int i = 0; i<14; i++) {
      point_array[i*4] = point_array_temp[i*4]; // x offset
      point_array[i*4+1] = point_array_temp[i*4+1]; // y offset
      // point_array[i*4+2] = point_array_temp[i*4+2]-1.2; // z offset
      point_array[i*4+2] = point_array_temp[i*4+2];
      point_array[i*4+3] = point_array_temp[i*4+3];
    }
    point_array[56]=point_array_temp[56];
    for (int i = 0; i < 12; ++i) state_feedback[i] = state_feedback_temp[i];
    // ROS_INFO("Np11 %.3f %.3f %.3f %.3f", point_array[44], point_array[45], point_array[46], point_array[47]);
    // prepare to send commands
    std_msgs::Float64MultiArray obstacle_data;
    obstacle_data.data.clear();
    // printf( "obstacle %i = %f \n", 1, point_array[0]);
    for (int i = 0; i < 57; i++){
      // printf( "obstacle %i = %f \n", i, point_array[i]);
      obstacle_data.data.push_back(point_array[i]);
    } 
    chatter_low.publish(obstacle_data);
    chatter_high.publish(obstacle_data);

    std_msgs::Float64MultiArray state_data;
    state_data.data.clear();
    // printf( "obstacle %i = %f \n", 1, point_array[0]);
    for (int i = 0; i < 12; i++) state_data.data.push_back(state_feedback[i]);
    for (int i = 0; i < 6; i++) state_data.data.push_back(from_high[18+i]);
    state_data.data.push_back(from_high[30]);
    states_low.publish(state_data);
    states_high.publish(state_data);

    // std_msgs::Float64MultiArray high_state_data;
    // high_state_data.data.clear();
    // for (int i = 0; i < 31; i++) high_state_data.data.push_back(from_high[i]);
    // chatter_pub.publish(high_state_data);


    ros::spinOnce();
    // loop_rate.sleep();
  }
  ros::spin();
  return 0;
}

