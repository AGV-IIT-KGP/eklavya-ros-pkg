#ifndef _PATH_PLANNER_H_
#define _PATH_PLANNER_H_

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <boost/bind.hpp>
#include "../../eklavya2.h"

namespace Nav {
  typedef struct command {
    int left_velocity, right_velocity;
  } command;
  
  class NavCore {
  public:
    static int mode;
    static void loadNavigator();
    static command navigate(Triplet target, int frameCount, double yaw);
    static void closeNav();
  };
}

#endif
