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
#include "../../eklavya2.h"

#define CENTERX 125
#define CENTERY 500
#define MAP_X 1000
#define MAP_Y 1000
#define HOKUYO_SCALE 100
#define VIEW_OBSTACLES 0
#define RADIUS 100

using namespace mrpt;
using namespace mrpt::hwdrivers;
using namespace mrpt::slam;
using namespace mrpt::gui;
using namespace mrpt::utils;
using namespace std;

LidarData::LidarData(string serial_name) {
    //laser.setSerialPort(serial_name);
    //if (!laser.turnOn()) {
        //printf("[TEST] Initialization failed!\n");
        //return;
    //}
}

int checksum(char **localmap, int x, int y) {
    int threshold = 3;
    int win_size = 4;
    int sum = 0;
    for (int i = x - win_size; i < x + win_size; i++) {
        for (int j = y - win_size; j < y + win_size; j++) {
            if (i < 950 - RADIUS && i > RADIUS + 50 && j > RADIUS && j < 875 - RADIUS) {
                if (j > CENTERX + 5) {
                    if (localmap[i][j] == 1)
                        sum++;
                }
            }
        }
    }
    if (sum >= threshold)
        return 1;
    else
        return 0;

}

void LidarData::createCircle(int x, int y, int R) {
    //TODO: optimize using circle drawing alogrithm    

    for (int i = -RADIUS; i < RADIUS; i++) {
        for (int j = -RADIUS; j < RADIUS; j++) {
            if (i * i + j * j <= RADIUS * RADIUS) {
                global_map[x + i][y + j] = 255;
            }
        }
    }
}

void plotMap(char **local_map) {
    IplImage *mapImg = cvCreateImage(cvSize(MAP_X, MAP_Y), IPL_DEPTH_8U, 1);

    for (int i = 0; i < MAP_X; i++) {
        uchar* ptr = (uchar *) (mapImg->imageData + i * mapImg->widthStep);
        for (int j = 0; j < MAP_Y; j++) {
            if (local_map[j][MAP_X - i - 1] == 0) {
                ptr[j] = 0;
            } else {
                ptr[j] = 200;
            }
        }
    }

    cvShowImage("Global Map", mapImg);
    cvWaitKey(100);
    //cvSaveImage("map.png", mapImg);
    cvReleaseImage(&mapImg);
}

void LidarData::update_map(const sensor_msgs::LaserScan& scan) {
  pthread_mutex_lock(&map_mutex);
  
  size_t size = scan.ranges.size();
  float angle = scan.angle_min;
  float maxRangeForContainer = scan.range_max - 0.1f;
  
  for (int i = 0; i < MAP_MAX; i++) {
    for (int j = 0; j < MAP_MAX; j++) {
      global_map[i][j] = 0;
    }
  }
  
  for (size_t i = 0; i < size; ++i)
  {
    float dist = scan.ranges[i];

    if ( (dist > scan.range_min) && (dist < maxRangeForContainer))
    {
      dist *= LIDAR_SCALE_TO_MAP;
      int x = cos(angle) * dist;
      int y = sin(angle) * dist;
      
      if (x < 900 - RADIUS && x > RADIUS + 50 && y > RADIUS && y < 800 - RADIUS && !(x < 560 && x > 440 && y < 150)) {
        if (y > CENTERX + 5) {
          createCircle(x, y, RADIUS);
        }
      }
    }

    angle += scan.angle_increment;
  }
  
  pthread_mutex_unlock(&map_mutex);
}

LidarData::~LidarData() {
    laser.turnOff();
}

