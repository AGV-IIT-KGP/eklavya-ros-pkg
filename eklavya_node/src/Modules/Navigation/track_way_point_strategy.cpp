#include <stdio.h>

#include "navigation.h"

namespace navigation_space {
  #define Lat_Dist 111119.99
  #define Long_Dist 102796.84
  
  double refN = 22.319575;
  double refE = 87.298412;

  void truncate(double xt, double yt, int *xtt, int *ytt)   {
    if ((yt <= 0.9 * MAP_MAX && yt >= 0.1 * MAP_MAX) && (xt <= 0.9 * MAP_MAX && xt >= 0.1 * MAP_MAX)) {
      *xtt = xt;
      *ytt = yt;
      return;
    }

    double xp, yp;
    int xb = 500;
    int yb = 200;
    
    // L1: y = 0.9A
    xp = xb + (0.9 * MAP_MAX - yb) * ((xt - xb) / (yt - yb));
    yp = 0.9 * MAP_MAX;
    if (((0.1 * MAP_MAX <= xp) && (xp <= 0.9 * MAP_MAX)) && ((yb - 0.9 * MAP_MAX) * (yt - 0.9 * MAP_MAX) <= 0) && (yt >= 0.9 * MAP_MAX)) {
      *xtt = xp;
      *ytt = yp;
      return;
    }

    // L2: y = 0.1A
    xp = xb + (0.1 * MAP_MAX - yb) * ((xt - xb) / (yt - yb));
    yp = 0.1 * MAP_MAX;

    if (((0.1 * MAP_MAX <= xp) && (xp <= 0.9 * MAP_MAX)) && ((yb - 0.1 * MAP_MAX) * (yt - 0.1 * MAP_MAX) <= 0) && (yt <= 0.1 * MAP_MAX)) {
      *xtt = xp;
      *ytt = yp;
      return;
    }

    // L3: x = 0.1A
    xp = 0.1 * MAP_MAX;
    yp = yb + (0.1 * MAP_MAX - xb) * ((yt - yb) / (xt - xb));
    if (yp > .1 * MAP_MAX && yp < .9 * MAP_MAX && (xt < 0.1 * MAP_MAX)) {
      *xtt = xp;
      *ytt = yp;
      return;
    }

    // L4: x = 0.9A
    xp = 0.9 * MAP_MAX;
    yp = yb + (0.9 * MAP_MAX - xb) * ((yt - yb) / (xt - xb));
    if (yp > .1 * MAP_MAX && yp < .9 * MAP_MAX && xt > 0.9 * MAP_MAX) {
      *xtt = xp;
      *ytt = yp;
      return;
    }
  }

  Triplet navigation_space::TrackWayPointStrategy::getTargetLocation(double x1, double y1, double heading) {
    heading = -(3.142 / 180.0) * heading;
    
    double x2 = x1 * cos(heading) - y1 * sin(heading);
    double y2 = y1 * cos(heading) + x1 * sin(heading);

    x2 *= 1;
    y2 *= -1;

    int tx, ty;
    truncate(x2 * 100, y2 * 100, &tx, &ty);
    
    Triplet target_location;
    target_location.x = tx;
    target_location.y = ty;
    target_location.z = 90;
    
    return target_location;
  }
  
  Triplet navigation_space::TrackWayPointStrategy::getBotLocation() {
    Triplet bot_location;
    bot_location.x = 0.5*MAP_MAX;
    bot_location.y = 0.1*MAP_MAX;
    bot_location.z = 90;
    return bot_location;
  }
}

