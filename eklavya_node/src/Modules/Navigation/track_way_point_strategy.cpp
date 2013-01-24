#include <stdio.h>

#include "navigation.h"

namespace navigation_space {
  Triplet navigation_space::TrackWayPointStrategy::getTargetLocation(double heading) {
    Triplet target_location;
    return target_location;
  }
  
  Triplet navigation_space::TrackWayPointStrategy::getBotLocation() {
    Triplet bot_location;
    bot_location.x = 0.5*MAP_MAX;
    bot_location.y = 0.1*MAP_MAX;
    bot_location.z = 0;
    return bot_location;
  }
}

