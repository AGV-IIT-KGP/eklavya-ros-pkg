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

  ros::Subscriber sub = gps_node.subscribe("fix", 1000, gps_space::GPS::updateLatLong);
  
  while(ros::ok()) {
    ros::spinOnce();
  }
}

