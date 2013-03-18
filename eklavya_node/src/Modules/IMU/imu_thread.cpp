#include <stdio.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "../../eklavya2.h"
#include "IMU.h"

void *imu_thread(void *arg) {
  int argc;
  char *argv[0];
  ros::init(argc, argv, "imu_thread");
  ros::NodeHandle imu_node;
  
  ros::Subscriber sub = imu_node.subscribe("/imu", 1000, IMUspace::IMU::update_pose);
  
  double heading = 0;
  while(ros::ok()) {
    ros::spinOnce();
  }
}

