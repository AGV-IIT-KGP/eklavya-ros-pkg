#include <stdio.h>
#include "../../eklavya2.h"
#include "diagnostics.h"

IplImage *map_image, *path_image;

void *diagnostics_thread(void *arg) {
  map_image = cvCreateImage(cvSize(MAP_MAX, MAP_MAX), IPL_DEPTH_8U, 3);
  path_image = cvCreateImage(cvSize(MAP_MAX, MAP_MAX), IPL_DEPTH_8U, 3);
  int iterations = 0;
  
  cvNamedWindow("Diag Path", 0);
  cvNamedWindow("Diag Map", 0);
  
  ros::Time::init();
  ros::Rate loop_rate(10);
  
  while(ros::ok()) {
      cout << iterations++ << endl;
      
    //pthread_mutex_lock(&pose_mutex);
    //diagnostics_space::Diagnostics::printPose();
    //pthread_mutex_unlock(&pose_mutex);
    
    //pthread_mutex_lock(&lat_long_mutex);
    //diagnostics_space::Diagnostics::printLatLong();
    //pthread_mutex_unlock(&lat_long_mutex);
    
    //pthread_mutex_lock(&odom_mutex);
    //diagnostics_space::Diagnostics::printOdom();
    //pthread_mutex_unlock(&odom_mutex);
    
    pthread_mutex_lock(&map_mutex);
    diagnostics_space::Diagnostics::plotMap();
    pthread_mutex_unlock(&map_mutex);
    
    //pthread_mutex_lock(&bot_location_mutex);
    //diagnostics_space::Diagnostics::printBotLocation();
    //pthread_mutex_unlock(&bot_location_mutex);
    
    //pthread_mutex_lock(&target_location_mutex);
    //diagnostics_space::Diagnostics::printTargetLocation();
    //pthread_mutex_unlock(&target_location_mutex);
    
    
    pthread_mutex_lock(&path_mutex);
    
    vector<Triplet> my_path;
    for(int i = 0; i < path.size(); i++) {
      my_path.push_back(path[i]);
    }
    
    pthread_mutex_unlock(&path_mutex);
    diagnostics_space::Diagnostics::plotPath(my_path);
    
    loop_rate.sleep();
  }
}

