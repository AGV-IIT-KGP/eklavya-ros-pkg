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

using namespace std;

namespace gps_space {
  void GPS::updateLatLong(const sensor_msgs::NavSatFix& fix) {
    pthread_mutex_lock(&lat_long_mutex);
    
    lat_long.latitude = fix.latitude;
    lat_long.longitude = fix.longitude;
    
    //cout << "[INFO] [GPS] : " << lat_long.latitude << " " << lat_long.longitude << endl; 
    printf("[INFO] [GPS] : %lf %lf\n", lat_long.latitude, lat_long.longitude);
    
    pthread_mutex_unlock(&lat_long_mutex);
  }
}
