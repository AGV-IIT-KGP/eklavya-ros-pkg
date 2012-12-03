/* 
 * File:   LidarData.h
 * Author: bhuvnesh
 *
 * Created on 18 September, 2012, 6:56 PM
 */
#include <mrpt/hwdrivers/CHokuyoURG.h>
#include <mrpt/hwdrivers/CSerialPort.h>
#include <mrpt/gui.h>
#include <mrpt/maps.h>
#include <opencv/cv.h>
#include <opencv/cvaux.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>

using namespace mrpt;
using namespace mrpt::hwdrivers;
using namespace mrpt::slam;
using namespace mrpt::gui;
using namespace mrpt::utils;
using namespace std;

#ifndef LIDARDATA_H
#define	LIDARDATA_H

class LidarData {
public:
    LidarData(string serial_name);
    char** plotLaserScan(char **localmap);
    virtual ~LidarData();
private:
    char **checkObstacles(char **localmap, CSimplePointsMap map);
    CHokuyoURG laser;
    char **expandObstacles(char **localmap, CSimplePointsMap map);
    void createCircle(char **localmap, int x, int y, int R);
};

#endif	/* LIDARDATA_H */

