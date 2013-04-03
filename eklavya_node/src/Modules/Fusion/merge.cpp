#include "merge.h"

void Merge::laneLidar() {
    pthread_mutex_lock(&map_mutex);
    pthread_mutex_lock(&cam_input_mutex);
    pthread_mutex_lock(&global_map_mutex);
    
    for (int i = 0; i < MAP_MAX; i++)
        for (int j = 0; j < MAP_MAX; j++) {
            if (cam_input[i][j] == 255 || g_laser_scan[i][j] == 255)
                global_map[i][j] = 255;
            else
                global_map[i][j] = 0;
        }
    

    pthread_mutex_unlock(&global_map_mutex);
    pthread_mutex_unlock(&cam_input_mutex); 
    pthread_mutex_unlock(&map_mutex);
    
}
