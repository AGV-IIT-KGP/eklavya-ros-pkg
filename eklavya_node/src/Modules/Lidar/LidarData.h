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
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/LaserScan.h>

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
    static void update_map(const sensor_msgs::LaserScan&);
    virtual ~LidarData();
    
    
private:
    CHokuyoURG laser;
    static void createCircle(int x, int y, int R);
    
};

#endif	/* LIDARDATA_H */

