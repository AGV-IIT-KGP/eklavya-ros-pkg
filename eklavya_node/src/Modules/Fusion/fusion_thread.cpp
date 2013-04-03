#include "fusion.h"

void *fusion_thread(void *arg) {
  ros::Rate loop_rate(LOOP_RATE);
  while(ros::ok()) {
      Fusion fuse;
      fuse.laneLidar();
    loop_rate.sleep();
  }

  ROS_INFO("Fusion module exiting");
  return NULL;
}
