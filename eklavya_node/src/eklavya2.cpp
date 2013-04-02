#include "eklavya2.h"

#include "Modules/Diagnostics/diagnostics.h"
#include "Modules/EKF/ekf.h"
#include "Modules/Encoder/encoder.h"
#include "Modules/GPS/gps.h"
#include "Modules/IMU/IMU.h"
#include "Modules/Lidar/LidarData.h"
#include "Modules/Lane/lane_data.h"
#include "Modules/Navigation/navigation.h"
#include "Modules/Planner/planner.h"
#include "Modules/SLAM/slam.h"

//#define DIAG

using namespace cv;

/* Global data structures to be shared by all threads */
Pose pose; // Shared by IMU, EKF
LatLong lat_long; // Shared by GPS, EKF
Odom odom; // Shared by Encoder, EKF
unsigned char g_laser_scan[MAP_MAX][MAP_MAX]; // Shared by Lidar, Planner
unsigned char cam_input[MAP_MAX][MAP_MAX]; // by Camera for lane
unsigned char global_map[MAP_MAX][MAP_MAX]; // by Camera for lane

Triplet bot_location; // Shared by EKF, Planner
Triplet target_location; // Shared by EKF, Planner
vector<Triplet> path;

int strategy;
LidarData *laser;

/* mutex for mutually exclusive updating of the shared data structures */
pthread_mutex_t pose_mutex;
pthread_mutex_t lat_long_mutex;
pthread_mutex_t odom_mutex;
pthread_mutex_t map_mutex;
pthread_mutex_t bot_location_mutex;
pthread_mutex_t target_location_mutex;
pthread_mutex_t path_mutex;
pthread_mutex_t cam_input_mutex;

void createMutex() {
    pthread_mutex_init(&pose_mutex, NULL);
    pthread_mutex_init(&lat_long_mutex, NULL);
    pthread_mutex_init(&odom_mutex, NULL);
    pthread_mutex_init(&map_mutex, NULL);
    pthread_mutex_init(&bot_location_mutex, NULL);
    pthread_mutex_init(&target_location_mutex, NULL);
    pthread_mutex_init(&path_mutex, NULL);
    pthread_mutex_init(&cam_input_mutex, NULL);

    pthread_mutex_trylock(&pose_mutex);
    pthread_mutex_unlock(&pose_mutex);

    pthread_mutex_trylock(&lat_long_mutex);
    pthread_mutex_unlock(&lat_long_mutex);

    pthread_mutex_trylock(&odom_mutex);
    pthread_mutex_unlock(&odom_mutex);

    pthread_mutex_trylock(&map_mutex);
    pthread_mutex_unlock(&map_mutex);

    pthread_mutex_trylock(&bot_location_mutex);
    pthread_mutex_unlock(&bot_location_mutex);

    pthread_mutex_trylock(&target_location_mutex);
    pthread_mutex_unlock(&target_location_mutex);

    pthread_mutex_trylock(&path_mutex);
    pthread_mutex_unlock(&path_mutex);

 pthread_mutex_trylock(&cam_input_mutex);
    pthread_mutex_unlock(&cam_input_mutex);
}

void startThread(pthread_t *thread_id, pthread_attr_t *thread_attr, void *(*thread_name) (void *)) {
    if (pthread_create(thread_id, thread_attr, thread_name, NULL)) {
        ROS_ERROR("Unable to create thread");
        pthread_attr_destroy(thread_attr);
        exit(1);
    }
    sleep(1);
}

void startThreads() {
    pthread_attr_t attr;
    pthread_t imu_id, lidar_id,lane_id,  gps_id, slam_id, navigation_id, planner_id, diagnostics_id;

    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

    /* Create threads */

    switch (strategy) {
        case FollowNose: // Follow a straight line while avoiding obstacles
            startThread(&imu_id, &attr, &imu_thread);
            startThread(&lidar_id, &attr, &lidar_thread);
            startThread(&navigation_id, &attr, &navigation_thread);
            startThread(&planner_id, &attr, &planner_thread);
            break;

        case TrackWayPoint: // Reach a target GPS waypoint while avoiding obstacles
            startThread(&imu_id, &attr, &imu_thread);
            startThread(&gps_id, &attr, &gps_thread);
            startThread(&lidar_id, &attr, &lidar_thread);
            startThread(&navigation_id, &attr, &navigation_thread);
            startThread(&planner_id, &attr, &planner_thread);
            break;

        case HectorSLAM:
            startThread(&imu_id, &attr, &imu_thread);
            startThread(&lidar_id, &attr, &lidar_thread);
            startThread(&slam_id, &attr, &slam_thread);
            break;

        case LaserTestOnly:
            startThread(&lidar_id, &attr, &lidar_thread);
            startThread(&planner_id, &attr, &planner_thread);
            break;

        case PlannerTestOnly:
            //            startThread(&lidar_id, &attr, &lidar_thread);
            startThread(&lane_id, &attr, &lane_thread);
            startThread(&navigation_id, &attr, &navigation_thread);
            startThread(&planner_id, &attr, &planner_thread);
            break;

	case Fusion:
            startThread(&lane_id, &attr, &lane_thread);
//            startThread(&merge_id, &attr, &merge_thread);
            break;
    }

#ifdef DIAG  
    startThread(&diagnostics_id, &attr, &diagnostics_thread);
#endif

    pthread_attr_destroy(&attr);
}

void fin() {
    planner_space::Planner::finBot();
}

void init() {
    createMutex();
    ROS_INFO("Mutexes created");
}

void printUsage() {
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "eklavya_node");
    ros::Time::init();

    if (argc < 2) {
        ROS_ERROR("Incorrect input format");
        printUsage();
        ros::shutdown();
    } else {
        strategy = atoi(argv[1]);
        ROS_INFO("Using the strategy: %d", strategy);
    }

    init();
    ROS_INFO("Eklavya initiated successfully");

    startThreads();
    ROS_INFO("Threads have been started");

    ROS_INFO("Spinning");
    ros::spin();

    ROS_INFO("Eklavya Exiting");

    return 0;
}
