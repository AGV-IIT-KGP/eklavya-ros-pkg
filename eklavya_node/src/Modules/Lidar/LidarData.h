/* 
 * File:   LidarData.h
 * Author: bhuvnesh
 *
 * Created on 18 September, 2012, 6:56 PM
 */
#include "../../eklavya2.h"
#include <opencv2/core/types_c.h>
#include <cvblob.h>
#include "sensor_msgs/LaserScan.h"

using namespace std;

#ifndef LIDARDATA_H
#define	LIDARDATA_H

class LidarData {
public:
    LidarData(string serial_name);
    static void update_map(const sensor_msgs::LaserScan&);
    virtual ~LidarData();

private:
    static void createCircle(int x, int y);

};

using namespace cvb;

#endif	/* LIDARDATA_H */

