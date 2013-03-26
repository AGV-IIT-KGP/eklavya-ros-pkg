#include "planner.h"

//#define FPS_TEST

char **local_map;
IplImage *map_img;

void *planner_thread(void *arg) {
    Triplet my_bot_location;
    Triplet my_target_location;

    map_img = cvCreateImage(cvSize(MAP_MAX, MAP_MAX), IPL_DEPTH_8U, 3);
    cvNamedWindow("[PLANNER] Map", 0);

    //initializing local map
    local_map = new char*[MAP_MAX];
    for (int i = 0; i < MAP_MAX; i++) {
        local_map[i] = new char[MAP_MAX];
    }

    printf("Initiating Planner\n");
    planner_space::Planner::loadPlanner();
    printf("Planner Initiated\n");

    cout << "Waiting for Target" << endl;
    usleep(999999);
    usleep(999999);

#ifdef FPS_TEST
    int iterations = 0;
    time_t start = time(0);
#endif

    ros::Time::init();
    ros::Rate loop_rate(10);

    while (1) {
#ifdef FPS_TEST
        if (iterations > 1000) {
            time_t finish = time(0);
            double fps = (iterations + 0.0) / (finish - start);
            cout << "[INFO] ITERATIONS: " << iterations << endl;
            cout << "[INFO] FPS: " << fps << endl;
            break;
        }
        iterations++;
#endif

#ifdef FPS_TEST
        my_bot_location.x = 500;
        my_bot_location.y = 100;
        my_bot_location.z = 90;
#else
        pthread_mutex_lock(&bot_location_mutex);
        my_bot_location = bot_location; // Bot
        pthread_mutex_unlock(&bot_location_mutex);
#endif

#ifdef FPS_TEST
        srand(rand() * time(0));
        double randx = 500;
        double randy = 900;

        //randx = 100 + rand() % 800; randy = 900;

        my_target_location.x = randx;
        my_target_location.y = randy;
        my_target_location.z = 90;
#else
        pthread_mutex_lock(&target_location_mutex);
        my_target_location = target_location; // Target
        pthread_mutex_unlock(&target_location_mutex);
#endif

        pthread_mutex_lock(&map_mutex);
        for (int i = 0; i < MAP_MAX; i++) {
            for (int j = 0; j < MAP_MAX; j++) {
                int i1 = i;
                int j1 = MAP_MAX - 1 - j;
                uchar* ptr = (uchar *) (map_img->imageData + j1 * map_img->widthStep);
                ptr[3 * i1 + 0] = global_map[i][j];
                ptr[3 * i1 + 1] = global_map[i][j];
                ptr[3 * i1 + 2] = global_map[i][j];
                local_map[i][j] = global_map[i][j];
            }
        }
        pthread_mutex_unlock(&map_mutex);

        cvShowImage("[PLANNER] Map", map_img);
        cvWaitKey(1);

        planner_space::Planner::findPath(my_bot_location, my_target_location);

        loop_rate.sleep();
    }

    cout << "Planner Exited" << endl;

    return NULL;
}

