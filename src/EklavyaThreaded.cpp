#include <iostream>
#include <time.h>
#include <pthread.h>
#include <sys/stat.h>
#include <sys/types.h>
#include "opencv/cv.h"
#include "opencv2/highgui/highgui.hpp"

#define MAP_MAX 1000

/// Comment to disable the following modules. Do not change the value 1

//#define _SON 1
//#define _GPSF 1
//#define _LANE 1
#define _NAV 1
//#define _STRO 1
#define _IMU 1
#define _LDR 1

/// Don't modify the code below this line

#ifdef _LANE
  #include "Lane.h"
  #define LANE_CHOICE 1 // 0 FOR HOUGH_LANES ONLY; 1 FOR COLOR_LANES ONLY; 2 FOR BOTH COLOR AND HOUGH LANES
#endif

#ifdef _IMU
  #include "IMU.h"
#endif

#ifdef _GPSF
  #include "GPS.h"
#endif

#ifdef _NAV
  #include "PathPlanner.h"
#endif

#ifdef _SON
#endif

#ifdef _STRO
  #include "Stereo.h"
#endif

#ifdef _LDR
  #include "LidarData.h"
#endif

using namespace cv;

pthread_t lidarThread, imuThread, plannerThread;
pthread_mutex_t csmutex;
pthread_mutex_t donemutex;
pthread_cond_t donecond;
typedef struct {
  char threadName[10];
} ThreadParams;

bool loggerActive = false;
char *logDirectory;
FILE *logFileHandle;
char **local_map;
CvPoint bot_loc;

#ifdef _LANE
  CvCapture *capture;
  IplImage* frame_in;
#endif

#ifdef _LDR
  LidarData *laser;
#endif

void createMutex() {
  printf("Creating Mutexes\n");

  pthread_mutex_init(&csmutex, NULL);
  pthread_mutex_init(&donemutex, NULL);

  pthread_mutex_trylock(&csmutex);
  pthread_mutex_unlock(&csmutex);

  pthread_mutex_trylock(&donemutex);
  pthread_mutex_unlock(&donemutex);

  pthread_cond_init(&donecond, NULL);
}

void destroyMutex() {
  printf("Destroying Mutexes\n");

  pthread_mutex_destroy(&csmutex);
  pthread_mutex_destroy(&donemutex);
  pthread_cond_destroy(&donecond);
}

void LidarThread()
{
  pthread_mutex_lock(&donemutex);
  pthread_cond_wait(&donecond, &donemutex);

  char **lidarMap = (char **)malloc(MAP_MAX * sizeof(char *));
  for(int i = 0; i < MAP_MAX; i++)
  {
    lidarMap[i] = (char *)malloc(MAP_MAX * sizeof(char));
  }

  while(1)
  {
    pthread_mutex_lock(&exitMutex);
    if(exitSignal == true)
    {
      pthread_mutex_unlock(&exitMutex);

      for(int i = 0; i < MAP_MAX; i++)
      {
        free(lidarMap + i);
      }
      free(lidarMap);

      pthread_exit(NULL);
    }

    plotLaserScan(lidarMap);

    pthread_mutex_lock(&mapMutex);
    for(int i = 0; i < MAP_MAX; i++)
    {
      for(int j = 0; j < MAP_MAX; j++)
      {
        local_map[i][j] = lidarMap[i][j];
      }
    }
    pthread_mutex_unlock(&mapMutex);
  }
}

void ImuThread()
{
  pthread_mutex_lock(&donemutex);
  pthread_cond_wait(&donecond, &donemutex);

  double imuYaw;

  while(1)
  {
    pthread_mutex_lock(&exitMutex);
    if(exitSignal == true)
    {
      pthread_mutex_unlock(&exitMutex);
      pthread_exit(NULL);
    }

    getYaw(&imuYaw);

    pthread_mutex_lock(&yawMutex);
    yaw = imuYaw;
    pthread_mutex_unlock(&yawMutex);
  }
}

void PlannerThread()
{
  pthread_mutex_lock(&donemutex);
  pthread_cond_wait(&donecond, &donemutex);

  char **plannerMap = (char **)malloc(MAP_MAX * sizeof(char *));
  for(int i = 0; i < MAP_MAX; i++)
  {
    plannerMap[i] = (char *)malloc(MAP_MAX * sizeof(char));
  }

  while(1)
  {
    pthread_mutex_lock(&exitMutex);
    if(exitSignal == true)
    {
      pthread_mutex_unlock(&exitMutex);

      for(int i = 0; i < MAP_MAX; i++)
      {
        free(plannerMap + i);
      }
      free(plannerMap);

      pthread_exit(NULL);
    }

    pthread_mutex_lock(&mapMutex);
    for(int i = 0; i < MAP_MAX; i++)
    {
      for(int j = 0; j < MAP_MAX; j++)
      {
        plannerMap[i][j] = local_map[i][j];
      }
    }
    pthread_mutex_unlock(&mapMutex);

    pthread_mutex_lock(&targetMutex);
    
    navigate(plannerMap, targetLocation, iterations);
  }
}

void createThreads() {
  int i, j;
  pthread_attr_t attr;
  ThreadParams params[3];

  pthread_attr_init(&attr);
  pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

  if(pthread_create(&lidarThread, &attr, LidarMain, (void *)(params + 0))) {
    printf("[ERROR] Master thread: Unable to create Lidar thread\n");
    pthread_attr_destroy(&attr);
    exit(1);
  }

  if(pthread_create(&imuThread, &attr, ImuMain, (void *)(params + 1))) {
    printf("[ERROR] Master thread: Unable to create IMU thread\n");
    pthread_attr_destroy(&attr);
    exit(1);
  }

  if(pthread_create(&plannerThread, &attr, PlannerMain, (void *)(params + 2))) {
    printf("[ERROR] Master thread: Unable to create Planner thread\n");
    pthread_attr_destroy(&attr);
    exit(1);
  }

  sleep(1);

  pthread_attr_destroy(&attr);
}

void initializeAGV() {
  printf("Initializing Eklavya\n");

  createMutex();
  createThreads();

  bot_loc.x = int(0.5 * MAP_MAX);
  bot_loc.y = int(0.1 * MAP_MAX);

  local_map = (char**)malloc(MAP_MAX * sizeof(char*));
  for(int i = 0; i < MAP_MAX; i++) {
    local_map[i] = (char*)calloc(MAP_MAX, sizeof(char));
  }

  #ifdef _LANE
    capture = cvCreateFileCapture("clip0.avi");// From File
    //capture = cvCaptureFromCAM(0);//From Camera.
    if(capture == NULL) {
      printf("[ERROR] Unable to capture frame \n");
    }
    frame_in = cvQueryFrame(capture);
    Lanespace::LaneDetection::initializeLaneVariables(frame_in);
  #endif

  #ifdef _IMU
    printf("Initiating IMU\n");
    IMUspace::IMU::initIMU();
    printf("IMU Initiated\n");
  #endif

  #ifdef _GPSF
    GPSspace::GPS::GPS_Init();
  #endif

  #ifdef _NAV
    printf("Initiating Navigator\n");
    Nav::NavCore::loadNavigator();
    printf("Navigator Initiated\n");
  #endif

  #ifdef _SON
    Sonarspace::Sonar::loadSonar("COM5", 19200);
  #endif

  #ifdef _STRO
    Stereospace::Stereo::initializeStereo();
  #endif

  #ifdef _LDR
    printf("Initializing Lidar\n");
    laser = new LidarData("ttyACM0");
    printf("Lidar Initiated\n");
  #endif

  printf("Eklavya Initiated\n");
  printf("=================\n");
}

void plotMap() {
  IplImage *mapImg = cvCreateImage(cvSize(MAP_MAX, MAP_MAX), IPL_DEPTH_8U, 1);

  for(int i = 0; i < MAP_MAX; i++) {
    uchar* ptr = (uchar *)(mapImg->imageData + i * mapImg->widthStep);
    for(int j = 0; j < MAP_MAX; j++) {
      if(local_map[j][MAP_MAX - i - 1] == 0) {
        ptr[j] = 0;
      }
      else {
        ptr[j] = 200;
      }
    }
  }

  cvNamedWindow("Global Map", 0);
  cvShowImage("Global Map", mapImg);
  //cvSaveImage("map.png", mapImg);
  cvReleaseImage(&mapImg);
}

void refreshMap() {
  for(int i = 0; i < MAP_MAX; i++) {
    for(int j = 0; j < MAP_MAX; j++) {
      local_map[i][j] = 0;
    }
  }
}

void closeAGV() {
  #ifdef _GPSF
    //GPSspace::GPS::closeGPS();
  #endif

  #ifdef _LDR
    laser->~LidarData();
  #endif

  #ifdef _IMU
    IMUspace::IMU::closeIMU();
  #endif

  #ifdef _NAV
    Nav::NavCore::closeNav();
  #endif

  #ifdef _STRO
    Stereospace::Stereo::closeStereo();
  #endif

  if (loggerActive) {
    fclose(logFileHandle);
  }

  if (pthread_join(lidarThread, NULL)) {
     printf("[ERROR] Unable to wait for Lidar thread\n");
  }

  if (pthread_join(imuThread, NULL)) {
     printf("[ERROR] Unable to wait for IMU thread\n");
  }

  if (pthread_join(plannerThread, NULL)) {
     printf("[ERROR] Unable to wait for Planner thread\n");
  }

  destroyMutex();
}

int main(int argc, char **argv) {
  int iterations = 0, c;
  int scale;
  int calib = 1;
  int count = 0;
  double latitude, longitude;
  double mapHeight = 0.875 * MAP_MAX;;
  double heading = 0;
  double referenceHeading = 0;
  time_t start, finish;
  IplImage *frame_in = NULL;

  if(argc == 2) {
    logDirectory = (char *)malloc(20 * sizeof(char));
    sprintf(logDirectory, "Logs/[%02d]Log", atoi(argv[1]));
    mkdir(logDirectory, 0777);

    char logFileName[30];
    strcpy(logFileName, "");
    strcat(logFileName, logDirectory);
    strcat(logFileName, "/text.log");
    logFileHandle = fopen(logFileName, "w");
    loggerActive = true;
    printf("Logging Active\n");
  }
  else {
    loggerActive = false;
  }

  #ifdef _LANE
    if(argc==1) {
      printf("[PARAM] Please provide the scale parameter\n");
      exit(1);
    }
    else {
      scale = atoi(argv[1]);
    }
  #endif

  initializeAGV();

  time(&start);
  while(1) {
    if(loggerActive) {
      fprintf(logFileHandle, "[%4d]: ", iterations);
    }

    refreshMap();

    /// Data Acquisition
    /// Stereo, Lidar: Depth Map
    /// Lane: Lanes
    /// IMU: Heading
    /// GPS: Latitude and Longitude

    #ifdef _STRO
      Stereospace::Stereo::runStereo(local_map);
    #endif

    #ifdef _LDR
      laser->plotLaserScan(local_map);
    #endif

    #ifdef _LANE
      frame_in = cvQueryFrame(capture);
      if(frame_in == NULL) {
        printf("End of Input Stream\n");
        break;
      }
      Lanespace::LaneDetection::markLane(frame_in, local_map, LANE_CHOICE, scale);
    #endif

    #ifdef _SON
    #endif

    #ifdef _IMU
      IMUspace::IMU::getYaw(&heading);
      printf("Heading: %f\n", heading);

      if(iterations < 5) {
        usleep(200 * 1000);
        referenceHeading = heading;
        iterations++;

        if(loggerActive) {
          fprintf(logFileHandle, "\n");
        }

        continue;
      }

      if(iterations == 5) {
        printf("Reference Heading: %lf\n", referenceHeading);
      }

      if(loggerActive) {
        fprintf(logFileHandle, "Heading: %lf | ", heading);
      }
    #endif

    #ifdef _GPSF
      GPSspace::GPS::_GPS(&latitude, &longitude);

      if(iterations < 35) {
        iterations++;

        if(loggerActive) {
          fprintf(logFileHandle, "\n");
        }

        continue;
      }
    #endif

    /// Target Acquisition

    int gps = 0, imu = 0, lane = 0;

    #ifdef _GPSF
      gps = 1;
    #endif

    #ifdef _IMU
      imu = 1;
    #endif

    #ifdef _LANE
      lane = 1;
    #endif

    int targetId = gps * (1 << 2) + lane * (1 << 1) + imu;
    CvPoint targetLocation;

    switch(targetId) {
      case 0:
        // Dummy Target
        //targetLocation = cvPoint(0.5 * MAP_MAX, 0.95 * MAP_MAX);
        srand(time(0));
        targetLocation = cvPoint(100 + rand() % 700, 200 + rand() % 700);
        break;

      case 1: {
        // Target from IMU
        double alpha;

        if(heading * referenceHeading > 0) {
          alpha = referenceHeading - heading;
        }
        else {
          if(heading - referenceHeading > 180) {
            alpha = 360 + referenceHeading - heading;
          }
          else if(referenceHeading - heading > 180) {
            alpha = referenceHeading - heading - 360;
          }
          else {
            alpha = referenceHeading - heading;
          }
        }

        alpha *= 3.14 / 180;

        double beta = atan(0.4 * MAP_MAX / mapHeight);

        if((-beta <= alpha) && (alpha <= beta)) {
          targetLocation.x = mapHeight * tan(alpha) + 0.5 * MAP_MAX;
          targetLocation.y = mapHeight + 0.1 * MAP_MAX;
        }
        else if(alpha > beta) {
          targetLocation.x = 0.9 * MAP_MAX;
          targetLocation.y = 0.4 * MAP_MAX / tan(alpha) + 0.1 * MAP_MAX;
        }
        else if(alpha < -beta) {
          targetLocation.x = 0.1 * MAP_MAX;
          targetLocation.y = 0.1 * MAP_MAX - 0.4 * MAP_MAX / tan(alpha);
        }
        else {
          targetLocation.x = 0.5 * MAP_MAX;
          targetLocation.y = 0.1 * MAP_MAX;
        }

        break;
      }

      case 2:
        // Target from Lane
        break;

      case 3:
        // Target from Lane and IMU
        break;

      case 4:
        // Dummy Target
        targetLocation = cvPoint(0.5 * MAP_MAX, 0.95 * MAP_MAX);
        break;

      case 5:
        // Target from GPS and IMU
        //GPSspace::GPS::Local_Map_Coordinate(latitude, longitude, heading, &targetLocation.x, &targetLocation.y);
        break;

      case 6:
        // Target from GPS and Lane
        break;

      case 7:
        // Target from GPS, IMU and Lane
        break;

      default:
        // Error
        printf("[ERROR]: Target not set\n");
        exit(0);
    }

    printf("[%d] : Target: (%d, %d)\n", iterations, targetLocation.x, targetLocation.y);

    /// Planning and Navigation

    #ifdef _NAV
      if(Nav::NavCore::navigate(local_map, targetLocation, iterations) == 1) {
        //mapHeight -= 0.25 * MAP_MAX;
      }
      else {
        mapHeight = 0.875 * MAP_MAX;
      }
    #endif

    if(loggerActive) {
      fprintf(logFileHandle, "\n");
    }

    plotMap();
    iterations++;

    if((c = cvWaitKey(1)) == 27) {
      break;
    }
  }

  time(&finish);
  printf("Number of iterations: %d\n",iterations);
  closeAGV();
  printf("FPS: %f\n", iterations / difftime(finish, start));

  return 0;
}
