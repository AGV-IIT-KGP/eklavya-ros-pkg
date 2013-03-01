#ifndef _PLANNER_H_
#define _PLANNER_H_

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <boost/bind.hpp>
#include "../../eklavya2.h"

extern char** local_map;

namespace planner_space {
  typedef struct command {
    int left_velocity, right_velocity;
  } command;
  
  class Planner {
  public:
    static void loadPlanner();
    static vector<Triplet> findPath(Triplet bot, Triplet target);
    static void closePlanner();
  };
}

#endif
