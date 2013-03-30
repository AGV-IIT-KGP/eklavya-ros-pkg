/*
 * GPS Processing Node
 * Subscribes to fix published by nmea_gps_driver
 * 
 * */

#include "gps.h"

void *gps_thread(void *arg) {
    ros::NodeHandle gps_node;
    ros::Subscriber sub = gps_node.subscribe("/pose", 1, gps_space::GPS::updatePose);
    ros::Rate loop_rate(LOOP_RATE);

    ROS_INFO("Started GPS thread");

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return NULL;
}

