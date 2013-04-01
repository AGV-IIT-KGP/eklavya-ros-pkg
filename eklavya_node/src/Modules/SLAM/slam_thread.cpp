#include "slam.h"

void *slam_thread(void *arg) {
    while (ros::ok()) {
        slam_space::Hector::foo();
        // Interact with other modules in this thread
        // Implement algo in the Hector class in slam_space in slam.cpp
    }

    return NULL;
}

