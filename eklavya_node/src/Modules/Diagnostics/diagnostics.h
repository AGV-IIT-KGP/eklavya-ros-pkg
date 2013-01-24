#ifndef _DIAGNOSTICS_H
#define _DIAGNOSTICS_H

#include "ros/ros.h"

namespace diagnostics_space {
	class Diagnostics {
	public:
    static void plotMap();
    static void printPose();
    static void printLatLong();
    static void printOdom();
    static void printBotLocation();
    static void printTargetLocation();
	};
}

#endif
