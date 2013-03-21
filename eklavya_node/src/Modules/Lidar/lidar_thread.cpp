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
  
  ros::Subscriber sub = lidar_node.subscribe("scan", 1000, LidarData::update_map);
  ros::Rate loop_rate(15);
  //ros::spin();
  
  while(ros::ok()) {
	  ros::spinOnce();
    loop_rate.sleep();
  }
}

