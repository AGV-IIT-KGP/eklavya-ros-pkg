/*
 * Lidar Processing Node
 * Subscribes to scan published by hokuyo_node
 * Publishes local_laser_map subscribed by navigation_node
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
  
  ros::Subscriber sub = lidar_node.subscribe("scan", 1000, LidarData::update_map);
  
  while(1) {
    ros::spinOnce();
  }
}

