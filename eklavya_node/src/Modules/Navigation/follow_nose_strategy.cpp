#include "navigation.h"

namespace navigation_space {
    double reference_heading;

    void navigation_space::FollowNoseStrategy::calibrateReferenceHeading(double heading, int iterations) {
        if (iterations < 5) {
            reference_heading = heading;
            usleep(200 * 1000);
        } else if (iterations == 5) {
            ROS_INFO("Ref Heading: %lf", reference_heading);
        }
    }

    Triplet navigation_space::FollowNoseStrategy::getTargetLocation(double heading) {
        double alpha;
        Triplet target_location;

        if (heading * reference_heading > 0) {
            alpha = reference_heading - heading;
        } else {
            if (heading - reference_heading > 180) {
                alpha = 360 + reference_heading - heading;
            } else if (reference_heading - heading > 180) {
                alpha = reference_heading - heading - 360;
            } else {
                alpha = reference_heading - heading;
            }
        }

        alpha *= 3.14 / 180;

        double map_height = 0.875 * MAP_MAX;
        double beta = atan(0.4 * MAP_MAX / map_height);

        if ((-beta <= alpha) && (alpha <= beta)) {
            target_location.x = map_height * tan(alpha) + 0.5 * MAP_MAX;
            target_location.y = map_height + 0.1 * MAP_MAX;
        } else if (alpha > beta) {
            target_location.x = 0.9 * MAP_MAX;
            target_location.y = 0.4 * MAP_MAX / tan(alpha) + 0.1 * MAP_MAX;
        } else if (alpha < -beta) {
            target_location.x = 0.1 * MAP_MAX;
            target_location.y = 0.1 * MAP_MAX - 0.4 * MAP_MAX / tan(alpha);
        } else {
            target_location.x = 0.5 * MAP_MAX;
            target_location.y = 0.1 * MAP_MAX;
        }
        target_location.z = 0;

        target_location.x = target_location.x < 100 ? 100 : target_location.x;
        target_location.y = target_location.y < 100 ? 100 : target_location.y;

        return target_location;
    }

    Triplet navigation_space::FollowNoseStrategy::getBotLocation() {
        Triplet bot_location;
        bot_location.x = 0.5 * MAP_MAX;
        bot_location.y = 0.1 * MAP_MAX;
        bot_location.z = 90;
        return bot_location;
    }
}

