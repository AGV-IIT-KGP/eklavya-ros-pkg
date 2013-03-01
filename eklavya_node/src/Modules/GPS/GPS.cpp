#include <iostream>
#include <mrpt/slam/CObservationGPS.h>
#include <mrpt/base.h>
#include <mrpt/slam.h>
#include <mrpt/hwdrivers/CGPSInterface.h>
#include <mrpt/otherlibs/stlplus/smart_ptr.hpp>
#include <mrpt/utils/CStream.h>
#include <mrpt/slam/CObservation.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/utils/CFileOutputStream.h>
#include <mrpt/utils/CFileInputStream.h>
#include <mrpt/system/os.h>
#include <mrpt/synch/CCriticalSection.h>
#include <mrpt/system/os.h>
#include <string>
#include <string.h>
//#include "../devices.h"
#include "GPS.h"
#define D 0.1
#define WAIT 50
#define VAR_X 5 
#define VAR_Y 7
#define VAR_I 0.01

#define Lat_Dist 111119.99
#define Long_Dist 102796.84
#define REF_NORTH 22.31666666666667
#define REF_EAST 87.3
#define MAP_MAX 1000
#define A 1000
#define GPS_COM_PORT "/dev/ttyUSB0"

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::hwdrivers;
using namespace mrpt::system;
using namespace mrpt::utils;

#ifdef _WIN32 /* _Win32 is usually defined by compilers targeting 32 or 64 bit Windows systems */
  #define GPS_COM  "COM11"
#else
  #include <unistd.h>
  #define GPS_COM GPS_COM_PORT
#endif

CGPSInterface gps;

double refN = 22.319575;
double refE = 87.298412;
  
//Declarations for IMU
string SERIAL_NAME;

namespace GPSspace {
  void truncate(double xt, double yt, int *xtt, int *ytt)   {
    if ((yt <= 0.9 * A && yt >= 0.1 * A) && (xt <= 0.9 * A && xt >= 0.1 * A)) {
      *xtt = xt;
      *ytt = yt;
      return;
    }

    double xp, yp;
    int xb = 500;
    int yb = 200;
    
    // L1: y = 0.9A
    xp = xb + (0.9 * A - yb) * ((xt - xb) / (yt - yb));
    yp = 0.9 * A;
    if (((0.1 * A <= xp) && (xp <= 0.9 * A)) && ((yb - 0.9 * A) * (yt - 0.9 * A) <= 0) && (yt >= 0.9 * A)) {
      *xtt = xp;
      *ytt = yp;
      return;
    }

    // L2: y = 0.1A
    xp = xb + (0.1 * A - yb) * ((xt - xb) / (yt - yb));
    yp = 0.1 * A;

    if (((0.1 * A <= xp) && (xp <= 0.9 * A)) && ((yb - 0.1 * A) * (yt - 0.1 * A) <= 0) && (yt <= 0.1 * A)) {
      *xtt = xp;
      *ytt = yp;
      return;
    }

    // L3: x = 0.1A
    xp = 0.1 * A;
    yp = yb + (0.1 * A - xb) * ((yt - yb) / (xt - xb));
    if (yp > .1 * A && yp < .9 * A && (xt < 0.1 * A)) {
      *xtt = xp;
      *ytt = yp;
      return;
    }

    // L4: x = 0.9A
    xp = 0.9 * A;
    yp = yb + (0.9 * A - xb) * ((yt - yb) / (xt - xb));
    if (yp > .1 * A && yp < .9 * A && xt > 0.9 * A) {
      *xtt = xp;
      *ytt = yp;
      return;
    }
  }

  void GPSspace::GPS::Local_Map_Coordinate(double north_d, double east_d, double yaw_d, int *tx, int *ty) {
    double y1 = ((double)refN - (double)north_d) * ((double)Lat_Dist); 
    double x1 = ((double)refE - (double)east_d) * ((double)Long_Dist); 
    printf("(x, y) : (%lf, %lf)\n", x1, y1);
    
    yaw_d = -(3.142 / 180.0) * yaw_d;
    double x2 = x1 * cos(yaw_d) - y1 * sin(yaw_d);
    double y2 = y1 * cos(yaw_d) + x1 * sin(yaw_d);

    x2 *= 1;
    y2 *= -1;

    truncate(x2 * 100, y2 * 100, tx, ty);
  }

  void GPSspace::GPS::GPS_Init() {
    //gps.setSerialPortName("/dev/sensors/GPS_GM_R900");    
    
    //return;
    
    string serName;
    
    if (mrpt::system::fileExists("../src/Modules/GPS/CONFIG_gps.ini")) {
      CConfigFile iniFile("../src/Modules/GPS/CONFIG_gps.ini");
      gps.loadConfig(iniFile, "GPS");
    } else {
      if (SERIAL_NAME.empty()) {
        serName = GPS_COM;
      } else {
        serName = SERIAL_NAME;
      }
      gps.setSerialPortName(serName);
    }
  }

  void GPSspace::GPS::_GPS(double *north_d , double *east_d) {
    FILE *f= os::fopen("gps_log.txt","wt");
    
    if (!f) {
      return;
    }
    
    CGenericSensor::TListObservations lstObs;
    CGenericSensor::TListObservations::iterator itObs;
          
    gps.doProcess();
    mrpt::system::sleep(500);
    
    gps.getObservations(lstObs);

    if (lstObs.empty()) {
      printf("[GPS] Waiting for data...\n");
    } else {
      for (itObs = lstObs.begin(); itObs != lstObs.end(); itObs++) {
        ASSERT_(itObs->second->GetRuntimeClass() == CLASS_ID(CObservationGPS));
        
        CObservationGPSPtr gpsData = CObservationGPSPtr(itObs->second);
        *north_d = gpsData->GGA_datum.latitude_degrees;
        *east_d =  gpsData->GGA_datum.longitude_degrees;         
        break;
      }
      
      lstObs.clear();
    }
    
    os::fclose(f);
  }
}
