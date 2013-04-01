#include "navigation.h"

namespace navigation_space {

    void truncate(double xt, double yt, int *xtt, int *ytt) {
        if ((yt <= 0.9 * MAP_MAX && yt >= 0.1 * MAP_MAX) && (xt <= 0.9 * MAP_MAX && xt >= 0.1 * MAP_MAX)) {
            *xtt = xt;
            *ytt = yt;
            return;
        }

        double xp, yp;
        int xb = 500;
        int yb = 100;

        if (yt < 0.1 * MAP_MAX) {
            *ytt = 0.05 * MAP_MAX;
            *xtt = (xt < xb) ? 0.25 * MAP_MAX : 0.75 * MAP_MAX;
            return;
        }

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
        heading *= (3.142 / 180.0);

        double alpha = -heading;
        double x2 = x1 * cos(alpha) + y1 * sin(alpha);
        double y2 = -x1 * sin(alpha) + y1 * cos(alpha);

        // Adjusting according to map's scale
        x2 *= 100;
        y2 *= 100;

        // Shifting to bot's center
        x2 += 500;
        y2 += 100;

        int tx, ty;
        truncate(x2, y2, &tx, &ty);

        Triplet target_location;
        target_location.x = tx;
        target_location.y = ty;
        target_location.z = 90;

        return target_location;
    }

    Triplet navigation_space::TrackWayPointStrategy::getBotLocation() {
        Triplet bot_location;
        bot_location.x = 0.5 * MAP_MAX;
        bot_location.y = 0.1 * MAP_MAX;
        bot_location.z = 90;
        return bot_location;
    }
}

