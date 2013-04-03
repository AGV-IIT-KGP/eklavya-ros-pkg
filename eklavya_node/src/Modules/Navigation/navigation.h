#ifndef _NAVIGATION_H_
#define _NAVIGATION_H_

#include "../../eklavya2.h"

namespace navigation_space {

    void truncate(double xt, double yt, int *xtt, int *ytt);

    class FollowNoseStrategy {
    public:
        static void calibrateReferenceHeading(double, int);
        static Triplet getTargetLocation(double);
        static Triplet getBotLocation();
    };

    class TrackWayPointStrategy {
    public:
        static Triplet getTargetLocation(double latitude, double longitude, double heading);
        static Triplet getBotLocation();
    };

    class IGVCBasicStrategy {
    public:
        static Triplet getTargetLocation(double latitude, double longitude, double heading);
        static Triplet getBotLocation();
    };
}

#endif
