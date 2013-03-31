#ifndef _GPS_H
#define _GPS_H

#include "../../eklavya2.h"
#include "geometry_msgs/Pose.h"

namespace gps_space {
  class GPS {
  public:
    static void updatePose(const geometry_msgs::Pose::ConstPtr _pose);
  };
}

#endif
