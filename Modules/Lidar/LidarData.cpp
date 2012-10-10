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
#define RADIUS 50
using namespace mrpt;
using namespace mrpt::hwdrivers;
using namespace mrpt::slam;
using namespace mrpt::gui;
using namespace mrpt::utils;
using namespace std;

LidarData::LidarData(string serial_name) {
    laser.setSerialPort(serial_name);
    printf("[TEST] Turning laser ON...\n");
    if (laser.turnOn())
        printf("[TEST] Initialization OK!\n");
    else {
        printf("[TEST] Initialization failed!\n");
        return;
    }

    //Initializing display variables
//    win3D.Create("Obstacle Window", 1000, 1000);
//    win3D.setCameraAzimuthDeg(140);
//    win3D.setCameraElevationDeg(20);
//    win3D.setCameraZoom(1.0);
//    win3D.setFOV(240);
//    win3D.setCameraPointingToPoint(2.5, 0, 0);
//    gl_points = mrpt::opengl::CPointCloud::Create();
//    gl_points->setPointSize(5);
//    {
//        mrpt::opengl::COpenGLScenePtr &scene = win3D.get3DSceneAndLock();
//        // Create the Opengl object for the point cloud:
//        scene->insert(gl_points);
//        scene->insert(mrpt::opengl::CGridPlaneXY::Create(-500, 500, -500, 500, 0, 100));
//        //scene->insert( mrpt::opengl::stock_objects::RobotPioneer());
//        scene->insert(mrpt::opengl::stock_objects::CornerXYZ());
//        win3D.unlockAccess3DScene();
//        win3D.repaint();
//    }
}

void LidarData::plotLaserScan(char **localmap) {
    bool thereIsObservation, hardError;
    CObservation2DRangeScan obs;
    float x, y;
    int points_count;
//    gl_points->clear();
    //Specific laser scanner "software drivers" must process here
    //new data from the I/O stream, and, if a whole scan has arrived
    //, return it. This method MUST BE CALLED in a timely fashion by
    //the user to allow the proccessing of incoming data. It can be 
    //run in a different thread safely.
    laser.doProcessSimple(thereIsObservation, obs, hardError);

    if (hardError)
        printf("[TEST] Hardware error=true!!\n");

    if (thereIsObservation) {
        obs.sensorPose = CPose3D(0, 0, 0);
        mrpt::slam::CSimplePointsMap theMap;
        theMap.insertionOptions.minDistBetweenLaserPoints = 0;
        theMap.insertObservation(&obs);
        points_count = theMap.getPointsCount();
        for (int i = 0; i < points_count; i++)
        {
            theMap.getPoint((size_t) i, x, y);
            int center_x = (-1 * y * 200) + CENTERY;
            int center_y = (x * 200) + CENTERX;
            // Remove only the two data points which represent the rods. Don't remove the rest of them.
            if(center_x < 1000 - RADIUS && center_x > RADIUS && center_y > RADIUS && center_y < 1000 - RADIUS && !(center_x < 560 && center_x > 440 && center_y < 150))
            {
              if (center_y > CENTERX + 5)
              {
                for(int j = -RADIUS; j < RADIUS; j++)
                {
                  for(int k = -RADIUS; k < RADIUS; k++)
                    if(k * k + j * j < RADIUS * RADIUS)
                      localmap[(int) (-1 * y * 200) + CENTERY + k][(int) (x * 200) + CENTERX + j] = 1;
                }
                
              }
            }
        }
        //        theMap.~CSimplePointsMap();
    }
//    if (VIEW_OBSTACLES) {
//        mrpt::slam::CSimplePointsMap map;
//        int i, j;
//        for (i = 0; i < MAP_Y; i++) {
//            for (j = 0; j < MAP_X; j++) {
//                if (localmap[i][j] != 0) {
//                    map.insertPointFast(j - 500, i - 500, 1);
//                }
//            }
//        }
//        win3D.get3DSceneAndLock();
//        gl_points->loadFromPointsMap(&map);
//        win3D.unlockAccess3DScene();
//        win3D.repaint();
//        //        map.~CSimplePointsMap();
//    }
    //        gl_points->clear();
//    obs.~CObservation2DRangeScan();
}

LidarData::~LidarData() {
    laser.turnOff();
}
