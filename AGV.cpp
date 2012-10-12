#include <time.h>
#include <iostream>
#include "opencv/cv.h"
#include "opencv2/highgui/highgui.hpp"
#include <sys/stat.h>
#include <sys/types.h>

// **********comment to disable the following features.. do not change the value 1**********
//#define _SON 1
//#define _GPSF 1   comment to disable
//#define _LANE 1
#define _NAV 1
//#define _STRO 1
#define _IMU 1
#define _LDR 1
#define MAP_MAX 1000
//*************************

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
#ifndef _WIN32
#include <unistd.h>
#define Sleep(i) usleep(i)
#endif

char **local_map;
CvPoint bot_loc, local_target;
double yaw5;
CvCapture *capture;
IplImage* frame_in;

#ifdef _LDR
LidarData *laser;
#endif

double tCal = 0;

void initializeAGV()
{
  printf("Initializing Eklavya\n");

  //Initializing the position of the vehicle
  bot_loc.x = int(0.5*MAP_MAX);
  bot_loc.y = int(0.1*MAP_MAX);

  //Allocating memory for the local_map
  local_map = (char**)malloc(MAP_MAX*sizeof(char*));
  for(int i=0; i<MAP_MAX; i++)
  {
    local_map[i] = (char*)calloc(MAP_MAX, sizeof(char));
  }

  //Initializing the various modules of the vehicle

#ifdef _LANE
  capture = cvCreateFileCapture("clip0.avi");// From File
  //capture = cvCaptureFromCAM(0);//From Camera.
  if(capture == NULL)
  {
    printf("\n \t\terror in capturing frame from video \n");
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
  Sonarspace::Sonar::loadSonar("COM5",19200);
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

void plotMap()
{
  IplImage *mapImg = cvCreateImage(cvSize(MAP_MAX, MAP_MAX), IPL_DEPTH_8U, 1);
  int i=0,j_0=0,j_1=1000;
  int thickness=1,lineType=8,shift=0;

  for(int i=0; i<MAP_MAX; i++)
  {
    uchar* ptr = (uchar *)(mapImg->imageData + i*mapImg->widthStep);
    for(int j=0; j<MAP_MAX; j++)
    {
      if(local_map[j][MAP_MAX-i-1] == 0)
        ptr[j] = 0;
      else
        ptr[j] = 200;
    }
  }
  /*
  for(i=0;i<1000;i+=50)
    {
      CvPoint pt1= cvPoint(i,j_0);
            CvPoint pt2= cvPoint(i,j_1);
      cvLine(mapImg,pt1,pt2,cvScalar(255,0,0),thickness,lineType,shift);
    }

    for(i=0;i<1000;i+=50)
    {
      CvPoint pt1= cvPoint(j_0,i);
            CvPoint pt2= cvPoint(j_1,i);
      cvLine(mapImg,pt1,pt2,cvScalar(255,0,0),thickness,lineType,shift);
    }
*/
  cvNamedWindow("Global Map",0);
  cvShowImage("Global Map", mapImg);
  //cvSaveImage("map.png", mapImg);
  cvReleaseImage(&mapImg);
}

void refreshMap()
{
  for(int i=0; i<MAP_MAX; i++)
  {
    for(int j=0; j<MAP_MAX; j++)
    {
      local_map[i][j] = 0;
    }
  }
}

void closeAGV()
{
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
}

int main(int argc, char **argv)
{
  time_t start,finish;
  int frame_count=0, c;
  double tx1, ty1;
  int tx, ty;
  IplImage *frame_in = NULL;
  int scale;
  int calib = 1;
  FILE *logFile;
  bool loggerActive = false;

  if(argc == 2)
  {
    //mkdir("Logs", 0777);

    char dirName[20];
    sprintf(dirName, "Logs/[%02d]Log", atoi(argv[1]));
    //sprintf(dirName, "[%02d]Log", atoi(argv[1]));
    mkdir(dirName, 0777);

    char fileName[30];
    sprintf(fileName, "Logs/[%02d]Log/text.log", atoi(argv[1]));
    //sprintf(fileName, "[%02d]Log/text.log", atoi(argv[1]));
    logFile = fopen(fileName, "w");
    loggerActive = true;
    printf("Logging Active\n");
  }
  else
    loggerActive = false;

#ifdef _LANE
  if(argc==1)
  {
    printf("provide the commandline - scale parameter \n");
    exit(1);
  }
  else
  {
    scale = atoi(argv[1]);
  }
#endif

  //Initializing the various modules of Vehicle
  initializeAGV();
  int li=0, target_set =1;
  local_target = cvPoint(500, 900);
  time(&start);
  int count = 0;
  while(1)
  {
    if(loggerActive)
    {
      fprintf(logFile, "[%4d]: ", frame_count);
    }

    //Cleaning the previous history present in the local map
    //Initializing each block to value 0
    //printf("map refreshing \n");
    refreshMap();
    //printf("map refreshed \n");
#ifdef _STRO
    Stereospace::Stereo::runStereo(local_map);
#endif

#ifdef _LDR
    //printf("loop %d inside lidar \n",count);
    laser->plotLaserScan(local_map);
    //printf("loop %d outside lidar \n",count++);
#endif

#ifdef _LANE
    frame_in = cvQueryFrame(capture);
    if(frame_in == NULL)
    {
        printf("\n \t\t End of Input Stream \n");
        break;
    }
    Lanespace::LaneDetection::markLane(frame_in, local_map, LANE_CHOICE, scale);
#endif

#ifdef _SON
    //  Sonarspace::Sonar::runSonar(local_map,int(0.5*MAP_MAX),int(0.1*MAP_MAX));
#endif

#ifdef _IMU
    IMUspace::IMU::getYaw(&yaw5);
    if(loggerActive)
    {
      fprintf(logFile, "Yaw: %lf | ", yaw5);
    }
    //printf("yaw: %lf\n", yaw5);
#endif

#ifdef _GPSF
    double yaw_copy=yaw5;
    GPSspace::GPS::_GPS(&tx1, &ty1);
    GPSspace::GPS::Local_Map_Coordinate(tx1,ty1,yaw_copy,&tx,&ty);

    printf("\t\t\ttarget: (%d, %d)\n", tx, ty);

    if(li < 35)
        li++;
    else
    {
        local_target = cvPoint(tx, ty);
        target_set = 1;
    }
#else
    local_target = cvPoint(500, 600);
    target_set = 1;
#endif

#ifdef _NAV
#ifdef _IMU
    double t = yaw5, a;

    if(frame_count < 5)
    {
      usleep(200 * 1000);
      tCal = yaw5;
      frame_count++;

      if(loggerActive)
      {
        fprintf(logFile, "\n");
      }

      continue;
    }

    if(frame_count == 5)
      printf("tCal: %lf\n", tCal);

    if(t * tCal > 0)
    {
      a = tCal - t;
    }
    else
    {
      if(t - tCal > 180)
        a = 360 + tCal - t;
      else if(tCal - t > 180)
        a = tCal - t - 360;
      else
        a = tCal - t;
    }

    a *= 3.14 / 180;

    double h = 0.8 * MAP_MAX;
    double a1 = atan(0.4 * MAP_MAX / h);
    if((-a1 <= a) && (a <= a1))
    {
      tx = h * tan(a) + 0.5 * MAP_MAX;
      ty = h + 0.1 * MAP_MAX;
    }
    else if(a > a1)
    {
      tx = 0.9 * MAP_MAX;
      ty = 0.4 * MAP_MAX / tan(a) + 0.1 * MAP_MAX;
    }
    else if(a < -a1)
    {
      tx = 0.1 * MAP_MAX;
      ty = 0.1 * MAP_MAX - 0.4 * MAP_MAX / tan(a);
    }
    else
    {
      tx = 0.5 * MAP_MAX;
      ty = 0.1 * MAP_MAX;
    }
    local_target = cvPoint(tx, ty);
    target_set = 1;
#endif
    if(loggerActive)
      Nav::NavCore::navigate(local_map, local_target, argv[1], logFile, frame_count);
    else
      Nav::NavCore::navigate(local_map, local_target);
#endif

    if(loggerActive)
    {
      fprintf(logFile, "\n");
    }

    plotMap();
    frame_count++;
    if((c = cvWaitKey(1)) == 27)
    {
      break;
    }
  }

if(loggerActive)
    fclose(logFile);

  time(&finish);
  printf("%d\n\n",frame_count);
  closeAGV();
  printf("fps: %f\n", frame_count / difftime(finish, start));
  return 0;
}
