#ifndef _DIAGNOSTICS_H
#define _DIAGNOSTICS_H

#include "../../eklavya2.h"

extern IplImage *map_image;
extern IplImage *path_image;

namespace diagnostics_space {

    class Diagnostics {
    public:
        static void plotMap();
        static void printPose();
        static void printLatLong();
        static void printOdom();
        static void printBotLocation();
        static void printTargetLocation();
        static void plotPath(std::vector<Triplet> my_path);
    };
}

#endif
