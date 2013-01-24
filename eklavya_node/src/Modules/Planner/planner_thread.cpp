#include <stdio.h>
#include "../../eklavya2.h"
#include "PathPlanner.h"

void *planner_thread(void *arg) {
  int iterations = 0;
  double heading;
  Triplet my_target_location;
  
  while(1) {
    pthread_mutex_lock(&target_location_mutex);
    my_target_location = target_location; // Target
    pthread_mutex_unlock(&target_location_mutex);
        
    pthread_mutex_lock(&pose_mutex);
    heading = pose.orientation.z; // Yaw
    pthread_mutex_unlock(&pose_mutex);
    
    Nav::NavCore::navigate(my_target_location, iterations, heading);
    
    iterations++;
  }
}

