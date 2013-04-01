/*
 * Lane Processing Node
 * Subscribes to scan published by hokuyo_node
 * 
 * */

#include <stdio.h>
#include "../../eklavya2.h"
#include "lane_data.h"
char **lane_scan;

void *lane_thread(void *arg) {
  int argc;
  char *argv[0]; 
  ros::init(argc, argv, "lane_thread");
  ros::NodeHandle lane_node;
  cvNamedWindow("view");
  cvNamedWindow("view_orig");
  cvNamedWindow("warp");
  
  LaneDetection lane_d;
  image_transport::ImageTransport it(lane_node);
  image_transport::Subscriber sub = it.subscribe("camera/image", 10, &LaneDetection::markLane, &lane_d);
  ros::Rate loop_rate(10);
  
  
  while(ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  printf("Exiting lane code");
}

