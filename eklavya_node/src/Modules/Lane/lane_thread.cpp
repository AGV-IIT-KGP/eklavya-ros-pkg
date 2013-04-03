#include "lane_data.h"

void *lane_thread(void *arg) {
  ros::NodeHandle lane_node;
  
  LaneDetection lane_d;
  image_transport::ImageTransport it(lane_node);
  image_transport::Subscriber sub = it.subscribe("camera/image", 2, &LaneDetection::markLane, &lane_d);
  ros::Rate loop_rate(LOOP_RATE);
  
  while(ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  ROS_INFO("Lane code exiting");
  
  return NULL;
}
