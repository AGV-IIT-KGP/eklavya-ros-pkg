/*
 * Lidar Processing Node
 * Subscribes to scan published by hokuyo_node
 * 
 * */

#include <stdio.h>
#include "../../eklavya2.h"
#include "LidarData.h"

char **laser_scan;

void *lidar_thread(void *arg) {
  int argc;
  char *argv[0];
  ros::init(argc, argv, "lidar_thread");
  ros::NodeHandle lidar_node;
  
  cvNamedWindow("Global Map", 0);
  
  ros::Subscriber sub = lidar_node.subscribe("scan", 1000, LidarData::update_map);
  
  while(ros::ok()) {
    ros::spinOnce();
  }
}

