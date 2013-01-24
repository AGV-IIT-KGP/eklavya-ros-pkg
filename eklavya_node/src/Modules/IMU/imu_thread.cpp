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
  
  

  double heading = 0;
  while(1) {
    //IMUspace::IMU::getYaw(&heading);
//    printf("Heading: %lf\n", heading);
    
    pthread_mutex_lock(&pose_mutex);
    pose.orientation.z = heading; // Yaw
    pthread_mutex_unlock(&pose_mutex);
  }
}

