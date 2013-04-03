#include "fusion.h"

void Fusion::laneLidar() {
    pthread_mutex_lock(&lidar_map_mutex);
    pthread_mutex_lock(&camera_map_mutex);
    pthread_mutex_lock(&global_map_mutex);

    for (int i = 0; i < MAP_MAX; i++)
        for (int j = 0; j < MAP_MAX; j++) {
            if (camera_map[i][j] == 255 || lidar_map[i][j] == 255) {
                global_map[i][j] = 255;
            } else {
                global_map[i][j] = 0;
            }
        }

    pthread_mutex_unlock(&global_map_mutex);
    pthread_mutex_unlock(&camera_map_mutex);
    pthread_mutex_unlock(&lidar_map_mutex);
}
