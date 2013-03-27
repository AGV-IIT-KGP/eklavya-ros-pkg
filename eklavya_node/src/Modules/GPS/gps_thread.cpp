/*
 * GPS Processing Node
 * Subscribes to fix published by nmea_gps_driver
 * 
 * */

#include <stdio.h>
#include "../../eklavya2.h"
#include "gps.h"

void *gps_thread(void *arg) {
    ros::NodeHandle gps_node;

    ros::Subscriber sub = gps_node.subscribe("/pose", 10, gps_space::GPS::updatePose);

    ros::Rate loop_rate(LOOP_RATE);
    
    while (1) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return NULL;
}

