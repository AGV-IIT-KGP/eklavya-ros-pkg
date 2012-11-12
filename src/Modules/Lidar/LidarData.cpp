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
#define RADIUS 100

using namespace mrpt;
using namespace mrpt::hwdrivers;
using namespace mrpt::slam;
using namespace mrpt::gui;
using namespace mrpt::utils;
using namespace std;

LidarData::LidarData(string serial_name) {
    laser.setSerialPort(serial_name);
    if (!laser.turnOn()) {
        printf("[TEST] Initialization failed!\n");
        return;
    }
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

char **LidarData::checkObstacles(char** localmap, CSimplePointsMap map) {
    char **obstacle_map;
    obstacle_map = (char **) calloc(1000, sizeof (char*));
    for (int i = 0; i < 1000; i++) {
        obstacle_map[i] = (char *) calloc(1000, sizeof (char));
    }
    int n_points = map.getPointsCount();
    float x,y;
    int i,j;
    for (int k = 0; k <n_points; k++) {
        map.getPoint((size_t)k,x,y);
        i = (int)((-1 * y * 200) + CENTERY);
        j = (int)((x * 200) + CENTERX);
        if (i < 900 - RADIUS && i > RADIUS + 50 && j > RADIUS && j < 700 - RADIUS &&
                    !(i < 560 && i > 440 && j < 150)) {
                if (j > CENTERX + 5) {
            
        if (checksum(localmap, i, j))
                obstacle_map[i][j] = 1;
            }
        }
    }
    return obstacle_map;
}

void LidarData::createCircle(char **obstacle_map, int x, int y, int R) {
    //TODO: optimize using circle drawing alogrithm    

    for (int i = -RADIUS; i < RADIUS; i++) {
        for (int j = -RADIUS; j < RADIUS; j++) {
            if (i * i + j * j <= RADIUS * RADIUS) {
                obstacle_map[x + i][y + j] = 1;
            }
        }
    }
}

char **LidarData::expandObstacles(char** localmap, CSimplePointsMap map) {
    int R = 100;
    char **obstacle_map = (char **) calloc(1000, sizeof (char *));
    for (int i = 0; i < 1000; i++) {
        obstacle_map[i] = (char *) calloc(1000, sizeof (char));
    }
    
    int n_points = map.getPointsCount();
    float x,y;
    int i,j;
    for (int k = 0; k <n_points; k++) {
        map.getPoint((size_t)k,x,y);
        i = (int)((-1 * y * 200) + CENTERY);
        j = (int)((x * 200) + CENTERX);
        if (i < 900 - RADIUS && i > RADIUS + 50 && j > RADIUS && j < 800 - RADIUS &&
                    !(i < 560 && i > 440 && j < 150)) {
                if (j > CENTERX + 5) {
                createCircle(obstacle_map, i, j, R);
            }
        }
    }

    for (int i = 0; i < 1000; i++) {
        free(localmap[i]);
    }
    free(localmap);
    return obstacle_map;
}

void Rotate(double inx, double iny, double *outx, double *outy, double theta) {
    theta *= CV_PI / 180;
    *outx = inx * sin(theta) + iny * cos(theta);
    *outy = -inx * cos(theta) + iny * sin(theta);
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

char** LidarData::plotLaserScan(char **localmap) {
    bool thereIsObservation=false, hardError;
    CObservation2DRangeScan obs;
    float x, y;
    int points_count;
	//laser.purgeBuffers();
	//while(!thereIsObservation)
		laser.doProcessSimple(thereIsObservation, obs, hardError);

    if (hardError) {
        printf("[TEST] Hardware error=true!!\n");
    }

    if (thereIsObservation) {
        obs.sensorPose = CPose3D(0, 0, 0);
        mrpt::slam::CSimplePointsMap theMap;
        theMap.insertionOptions.minDistBetweenLaserPoints = 0;
        theMap.insertObservation(&obs);
        points_count = theMap.getPointsCount();

        for (int i = 0; i < points_count; i++) {
            theMap.getPoint((size_t) i, x, y);

            double center_x = (-1 * y * 200) + CENTERY;
            double center_y = (x * 200) + CENTERX;

            //Rotate(center_x, center_y, &center_x, &center_y, 85);
            if (center_x < 900 - RADIUS && center_x > RADIUS + 50 && center_y > RADIUS && center_y < 700 - RADIUS &&
                    !(center_x < 560 && center_x > 440 && center_y < 150)) {
                if (center_y > CENTERX + 5) {
                    localmap[(int) center_x][(int) center_y] = 1;
                }
            }
        }
        //got points    
        //localmap = LidarData::checkObstacles(localmap, theMap);
        localmap = LidarData::expandObstacles(localmap, theMap);
        //plotMap(localmap);
        return localmap;
    }
}

LidarData::~LidarData() {
    laser.turnOff();
}

