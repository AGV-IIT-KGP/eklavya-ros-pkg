#include <stdio.h>
#include "../../eklavya2.h"
#include "GPS.h"

void *gps_thread(void *arg) {
  double latitude, longitude;
  
  while(1) {
    GPSspace::GPS::_GPS(&latitude, &longitude);
    pthread_mutex_lock(&pose_mutex);
    lat_long.latitude = latitude;
    lat_long.longitude = longitude;
    pthread_mutex_unlock(&pose_mutex);
  }
}

