#ifndef _NAVIGATION_H_
#define _NAVIGATION_H_

#include "../../eklavya2.h"

namespace navigation_space {
  class FollowNoseStrategy {
  public:
    static void calibrateReferenceHeading(double heading);
    static Triplet getTargetLocation(double heading);
    static Triplet getBotLocation();
	};
  
  class TrackWayPointStrategy {
  public:
    static Triplet getTargetLocation(double heading);
    static Triplet getBotLocation();
  };
}

#endif
