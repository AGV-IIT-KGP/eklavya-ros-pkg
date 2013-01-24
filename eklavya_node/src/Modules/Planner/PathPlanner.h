#ifndef _PATH_PLANNER_H_
#define _PATH_PLANNER_H_

#include "ros/ros.h"
#include "../../eklavya2.h"

namespace Nav {
  class NavCore {
  public:
    static int mode;
    static void loadNavigator();
    static int navigate(Triplet target, int frameCount, double yaw);
    static void closeNav();
  };
}

#endif
