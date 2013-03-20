#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <iostream>

#include "opencv/cv.h"
#include "opencv2/highgui/highgui.hpp"

#include "Modules/GPS/GPS.h"
#include "Modules/IMU/IMU.h"
#include "Modules/Lidar/LidarData.h"
#include "Modules/Encoder/encoder.h"
#include "Modules/Planner/planner.h"
#include "Modules/Diagnostics/diagnostics.h"
#include "ros/ros.h"
#include "eklavya2.h"

#define DIAG

using namespace cv;

/* Global data structures to be shared by all threads */
Pose pose; // Shared by IMU, EKF
LatLong lat_long; // Shared by GPS, EKF
Odom odom; // Shared by Encoder, EKF
unsigned char global_map[MAP_MAX][MAP_MAX]; // Shared by Lidar, Planner
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

void createMutex () {
  pthread_mutex_init(&pose_mutex, NULL);
  pthread_mutex_init(&lat_long_mutex, NULL);
  pthread_mutex_init(&odom_mutex, NULL);
  pthread_mutex_init(&map_mutex, NULL);
  pthread_mutex_init(&bot_location_mutex, NULL);
  pthread_mutex_init(&target_location_mutex, NULL);
  pthread_mutex_init(&path_mutex, NULL);
  
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
}

void startThread(pthread_t *thread_id, pthread_attr_t *thread_attr, void *(*thread_name) (void *)) {
  if (pthread_create(thread_id, thread_attr, thread_name, NULL)) {
    fprintf(stderr, "Unable to create thread\n");
    pthread_attr_destroy(thread_attr);
    exit(1);
  }
  sleep(1);
}

void startThreads() {
  pthread_attr_t attr;
  pthread_t imu_id, lidar_id, gps_id, encoder_id, ekf_id, slam_id, navigation_id, planner_id, diagnostics_id;
  
  pthread_attr_init(&attr);
  pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

  /* Create threads */
  
  switch(strategy) {
    case FollowNose: // Follow a straight line while avoiding obstacles
      startThread(&imu_id, &attr, &imu_thread);
      //startThread(&lidar_id, &attr, &lidar_thread);
      startThread(&navigation_id, &attr, &navigation_thread);
      startThread(&planner_id, &attr, &planner_thread);
      break;
      
    case TrackWayPoint: // Reach a target GPS waypoint while avoiding obstacles
      startThread(&imu_id, &attr, &imu_thread);
      //startThread(&gps_id, &attr, &gps_thread);
      //startThread(&lidar_id, &attr, &lidar_thread);
      startThread(&navigation_id, &attr, &navigation_thread);
      //startThread(&planner_id, &attr, &planner_thread);
      break;
      
    case HectorSLAM:
      startThread(&imu_id, &attr, &imu_thread);
      startThread(&lidar_id, &attr, &lidar_thread);
      startThread(&slam_id, &attr, &slam_thread);
      break;
    case LaserTestOnly:
	startThread(&lidar_id, &attr, &lidar_thread);
        //startThread(&planner_id, &attr, &planner_thread);
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

void sigHandler(int sig) {
  printf("\n Closing AGV\n");
  
  signal(sig, SIG_IGN);

  fin();
  
  exit(0);
}

void init() {
  printf("Initializing Eklavya\n");

  signal(SIGINT, sigHandler);
  
  printf("Initializing Lidar\n");
  //laser = new LidarData("ttyACM0");  
  printf("\tLidar Initiated\n");
    
  //printf("Initiating Encoder\n");
  //encoder_space::Encoder::initEncoder();
  //printf("Encoder Initiated\n");
  
  printf("Eklavya Initiated\n");
  printf("=================\n");
}

void printUsage() {
  printf("Incorrect Input Format\n");
}

int main(int argc, char *argv[]) {
  if (argc < 2) {
    printUsage();
    exit(0);
  } else {
    strategy = atoi(argv[1]);
    cout << "Using the strategy: " << strategy << endl;
  }
  
  createMutex();

  init();

  startThreads();

  while(ros::ok());
  
  return 0;
}
