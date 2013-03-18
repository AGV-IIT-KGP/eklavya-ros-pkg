/*
 * GPS Processing Node
 * Subscribes to fix published by nmea_gps_driver
 * 
 * */
 
#include <stdio.h>
#include "../../eklavya2.h"
#include "gps.h"

void *gps_thread(void *arg) {
  int argc;
  char *argv[0];
  ros::init(argc, argv, "gps_thread");
  ros::NodeHandle gps_node;

  ros::Subscriber sub = gps_node.subscribe("/pose", 10, gps_space::GPS::updatePose);
  
  while(ros::ok()) {
    ros::spinOnce();
  }
}

