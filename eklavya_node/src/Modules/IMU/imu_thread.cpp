#include <stdio.h>
#include "IMU.h"

void *imu_thread(void *arg) {
    ROS_INFO("Started IMU thread");
    
    ros::NodeHandle imu_node;
    
    ros::Subscriber sub = imu_node.subscribe("/yaw", 1, IMUspace::IMU::update_yaw);

    ros::Rate loop_rate(LOOP_RATE);

    while (1) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return NULL;
}

