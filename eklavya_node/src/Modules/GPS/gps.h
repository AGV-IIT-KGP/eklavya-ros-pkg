#ifndef _GPS_H
#define _GPS_H

#include "ros/ros.h"
#include <sensor_msgs/NavSatFix.h>

namespace gps_space {
  class GPS {
  public:
    static void updateLatLong(const sensor_msgs::NavSatFix&);
  };
}

#endif
