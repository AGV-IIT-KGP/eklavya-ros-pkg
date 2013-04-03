#include "../../eklavya2.h"

using namespace std;

extern unsigned char my_lidar_map[MAP_MAX][MAP_MAX];
extern unsigned char my_camera_map[MAP_MAX][MAP_MAX];

class Fusion {
public:
    void laneLidar();
};
