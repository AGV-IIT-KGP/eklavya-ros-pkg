#include <stdio.h>
#include "../../eklavya2.h"
#include "slam.h"

void *slam_thread(void *arg) {
  while(1) {
    slam_space::Hector::foo();
    // Interact with other modules in this thread
    // Implement algo in the Hector class in slam_space in slam.cpp
  }
}

