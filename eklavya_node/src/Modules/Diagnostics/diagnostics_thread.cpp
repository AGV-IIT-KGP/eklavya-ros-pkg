#include <stdio.h>
#include "../../eklavya2.h"
#include "diagnostics.h"

void *diagnostics_thread(void *arg) {
  int iterations = 0;
  while(ros::ok()) {
    printf("=========================================\n");
    printf("Diagnostics Iteration #: %d\n", iterations++);
    printf("=========================================\n");
    
    pthread_mutex_lock(&pose_mutex);
    diagnostics_space::Diagnostics::printPose();
    pthread_mutex_unlock(&pose_mutex);
    
    pthread_mutex_lock(&lat_long_mutex);
    diagnostics_space::Diagnostics::printLatLong();
    pthread_mutex_unlock(&lat_long_mutex);
    
    pthread_mutex_lock(&odom_mutex);
    diagnostics_space::Diagnostics::printOdom();
    pthread_mutex_unlock(&odom_mutex);
    
    pthread_mutex_lock(&map_mutex);
    diagnostics_space::Diagnostics::plotMap();
    pthread_mutex_unlock(&map_mutex);
    
    pthread_mutex_lock(&bot_location_mutex);
    diagnostics_space::Diagnostics::printBotLocation();
    pthread_mutex_unlock(&bot_location_mutex);
    
    pthread_mutex_lock(&target_location_mutex);
    diagnostics_space::Diagnostics::printTargetLocation();
    pthread_mutex_unlock(&target_location_mutex);
  }
}

