#ifndef _PLANNER_H_
#define _PLANNER_H_

#include "../../eklavya2.h"

extern char** local_map;
extern IplImage *map_img;

namespace planner_space {

    typedef struct command {
        int left_velocity, right_velocity;
    } command;

    class Planner {
    public:
        static void loadPlanner();
        static void findPath(Triplet bot, Triplet target);
        static void finBot();
    };
}

#endif
