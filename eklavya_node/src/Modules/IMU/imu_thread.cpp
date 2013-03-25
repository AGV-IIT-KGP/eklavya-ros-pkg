#include <stdio.h>
#include "IMU.h"

void *imu_thread(void *arg) {
    int argc;
    char *argv[0];
    ros::init(argc, argv, "imu_thread");
    ros::NodeHandle imu_node;

    ros::Subscriber sub = imu_node.subscribe("/yaw", 1000, IMUspace::IMU::update_yaw);

    ros::Rate loop_rate(10);

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return NULL;
}

