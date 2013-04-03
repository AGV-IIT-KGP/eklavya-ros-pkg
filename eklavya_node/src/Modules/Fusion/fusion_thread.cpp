#include "fusion.h"

unsigned char my_lidar_map[MAP_MAX][MAP_MAX];
unsigned char my_camera_map[MAP_MAX][MAP_MAX];

void *fusion_thread(void *arg) {
    ros::Rate loop_rate(LOOP_RATE);
    
    ROS_INFO("Fusion module started");
    
    while (ros::ok()) {
        Fusion fuse;
        fuse.laneLidar();
        loop_rate.sleep();
    }

    ROS_INFO("Fusion module exiting");
    return NULL;
}
