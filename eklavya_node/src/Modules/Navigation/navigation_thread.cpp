#include <stdio.h>
#include "../../eklavya2.h"
#include "navigation.h"

void *navigation_thread(void *arg) {
  double heading;
  Triplet my_target_location;
  Triplet my_bot_location;
  char walkability[MAP_MAX][MAP_MAX];
  
  while(1) {
    switch(strategy) {
      case FollowNose:
        pthread_mutex_lock(&pose_mutex);        
        heading = pose.orientation.z; // Yaw
        pthread_mutex_unlock(&pose_mutex);
        navigation_space::FollowNoseStrategy::calibrateReferenceHeading(heading);
        
        my_target_location = navigation_space::FollowNoseStrategy::getTargetLocation(heading);
        pthread_mutex_lock(&target_location_mutex);
        target_location = my_target_location; // Target
        pthread_mutex_unlock(&target_location_mutex);
        
        my_bot_location = navigation_space::FollowNoseStrategy::getBotLocation();
        pthread_mutex_lock(&bot_location_mutex);
        bot_location = my_bot_location; // Bot
        pthread_mutex_unlock(&bot_location_mutex);
        
        break;
        
      case TrackWayPoint:
        pthread_mutex_lock(&pose_mutex);        
        heading = pose.orientation.z; // Yaw
        pthread_mutex_unlock(&pose_mutex);
        
        my_target_location = navigation_space::TrackWayPointStrategy::getTargetLocation(heading);
        pthread_mutex_lock(&target_location_mutex);
        target_location = my_target_location; // Target
        pthread_mutex_unlock(&target_location_mutex);
        
        my_bot_location = navigation_space::TrackWayPointStrategy::getBotLocation();
        pthread_mutex_lock(&bot_location_mutex);
        bot_location = my_bot_location; // Bot
        pthread_mutex_unlock(&bot_location_mutex);
        
        break;
    }

    usleep(10);
  }
}

