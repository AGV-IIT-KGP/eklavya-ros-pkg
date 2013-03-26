#include <stdio.h>
#include "IMU.h"

void *imu_thread(void *arg) {
    ros::NodeHandle imu_node;
    
    ros::Subscriber sub = imu_node.subscribe("/yaw", 1000, IMUspace::IMU::update_yaw);

    ros::Rate loop_rate(10);

    while (1) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return NULL;
}

