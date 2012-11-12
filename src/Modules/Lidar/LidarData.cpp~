/*
 * File:   LidarData.cpp
 * Author: bhuvnesh
 *
 * Created on 18 September, 2012, 6:56 PM
 */

#include "LidarData.h"
#include <mrpt/hwdrivers/CHokuyoURG.h>
#include <mrpt/hwdrivers/CSerialPort.h>
#include <mrpt/gui.h>
#include <mrpt/maps.h>
#include <opencv2/core/types_c.h>

#define CENTERX 125
#define CENTERY 500
#define MAP_X 1000
#define MAP_Y 1000
#define HOKUYO_SCALE 100
#define VIEW_OBSTACLES 0
#define RADIUS 45

using namespace mrpt;
using namespace mrpt::hwdrivers;
using namespace mrpt::slam;
using namespace mrpt::gui;
using namespace mrpt::utils;
using namespace std;
using namespace cv;

LidarData::LidarData(string serial_name) {
  laser.setSerialPort(serial_name);
  if(!laser.turnOn()) {
    printf("[TEST] Initialization failed!\n");
    return;
  }
}

int sum(int x, int y, char**localmap)
{
  int sum =0;
  int i,j;
  for(i=-2;i<2;i++){
    for(j=-2;j<2;j++){
      sum += localmap[x+i][y+j];
    }
  }
  if(sum>3)
  return 1;
  else
  return 0;
}

void Rotate(double inx, double iny, double *outx, double *outy, double theta) {
  theta *= CV_PI / 180;
  *outx = inx * sin(theta) + iny * cos(theta);
  *outy = -inx * cos(theta) + iny * sin(theta);
}

int checksum(char **localmap, int x, int y)
{
    int threshold = 3;
    int win_size = 2;
    int sum = 0;
    for(int i=x-win_size; i<x+win_size;i++)
    {
        for(int j=y-win_size; j<y+win_size; j++)
        {
            if(localmap[i][j]==1)
                sum++;
        }
    }
    if(sum>=threshold)
        return 1;
    else
        return 0;
    
}

char **LidarData::checkObstacles(char** localmap){
    char **obstacle_map;
    obstacle_map = (char **)calloc(1000, sizeof(char*));
    for(int i=0;i<1000;i++)
    {
        obstacle_map[i] = (char *)calloc(1000,sizeof(char));
    }    
    int boundary = 100;

    for(int i=boundary;i< MAP_X-boundary;i++){
        for(int j=boundary;j<MAP_Y-boundary;j++){
            if(checksum(localmap,i,j))
                obstacle_map[i][j] = 1;
        }
    }

    /*for(int i=0;i<1000;i++)
    {
        free(localmap[i]);
    }
    free(localmap);*/
    return obstacle_map;
}

void LidarData::createCircle(char **obstacle_map, int x, int y, int R){
//TODO: optimize using circle drawing alogrithm    
R  = 60;
    for (int i = -R; i<R ; i++)
    {
        for(int j= -R; j<R; j++)
        {
            if(i*i + j*j <= R*R)
            {
                obstacle_map[x+i][y+j] = 1;
            }
        }
    }
}

char **LidarData::expandObstacles(char** localmap){
    int R = 100;
	char **obstacle_map  = (char **)calloc(1000,sizeof(char *));
	for(int i=0;i<1000;i++)
	{
		obstacle_map[i] = (char *)calloc(1000, sizeof(char));
	}
    for(int i=R; i<MAP_X-R; i++)
    {
        for(int j=R; j<MAP_Y-R; j++)
        {
            if(localmap[i][j]==1){
              createCircle(obstacle_map,i,j,R);
            }
        }
    }
for(int i=0;i<1000;i++)
	{
		free(localmap[i]);
	}
free(localmap);
return obstacle_map;
}

char** LidarData::plotLaserScan(char **localmap) {
  bool thereIsObservation, hardError;
  CObservation2DRangeScan obs;
  float x, y;
  int points_count;

  laser.doProcessSimple(thereIsObservation, obs, hardError);

  if(hardError) {
    printf("[TEST] Hardware error=true!!\n");
  }

  if(thereIsObservation) {
    obs.sensorPose = CPose3D(0, 0, 0);
    mrpt::slam::CSimplePointsMap theMap;
    theMap.insertionOptions.minDistBetweenLaserPoints = 0;
    theMap.insertObservation(&obs);
    points_count = theMap.getPointsCount();

    for(int i = 0; i < points_count; i++) {
      theMap.getPoint((size_t) i, x, y);
      localmap[(int) (-1 * y * 100) + CENTERY][(int) (x * 100) + CENTERX] = 1;
  }
  
  localmap = LidarData::checkObstacles(localmap);
   localmap = LidarData::expandObstacles(localmap);
return localmap;
  }
}

LidarData::~LidarData() {
  laser.turnOff();
}
