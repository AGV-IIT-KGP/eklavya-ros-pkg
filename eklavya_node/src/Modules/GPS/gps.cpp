/*
 * File:   gps.cpp
 * Author: Samuel
 *
 * Created on 10 Febraury, 2013, 1:35 AM
 */

#include <iostream>
#include <stdio.h>
#include "gps.h"
#include "../../eklavya2.h"
#include "geometry_msgs/Pose.h"

using namespace std;

namespace gps_space {
  void GPS::updatePose(const geometry_msgs::Pose::ConstPtr _pose) {
    pthread_mutex_lock(&pose_mutex);
    
    pose.position.x = _pose->position.x;
    pose.position.y = _pose->position.y;
    
    printf("[INFO] [POSE] : %lf %lf\n", pose.position.x, pose.position.y);
    
    pthread_mutex_unlock(&pose_mutex);
  }
}
