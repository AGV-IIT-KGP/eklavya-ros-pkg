#include<iostream>
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
#include "GPS.h"
#define D 0.1
#define WAIT 50
#define VAR_X 5
#define VAR_Y 7
#define VAR_I 0.01

#define Lat_Dist 111119.99
#define Long_Dist 102796.84                                     // This distance is at ****IIT Kharagpur ****.. ****Please update before leaving for ***IGVC***  ****
                                // for Oakland Universty : 
#define REF_NORTH 22.31666666666667                                        // 37.7481 : Oakland University :- Michigan
#define REF_EAST 87.3                      // -122.202 : Oakland University : Michigan
#define MAP_MAX 1000

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::hwdrivers;
using namespace mrpt::system;
using namespace mrpt::utils;

//Declarations for GPS
#ifdef _WIN32 /* _Win32 is usually defined by compilers targeting 32 or 64 bit Windows systems */
  //# include <tserial.h>
  #define GPS_COM  "COM11"
#else
//#elif defined __unix__ /* __unix__ is usually defined by compilers targeting Unix systems */
  #include <unistd.h>
  //#include "serial_lnx.h"
  #define GPS_COM "/dev/ttyUSB0"
#endif
CGPSInterface   gps;

double refN = 22.322091;
double refE = 87.306429;
  
//Declarations for IMU
string SERIAL_NAME;

namespace GPSspace
{
  void truncate(double xb, double yb, double xt, double yt, double *xtt, double **ytt)
  {
    double xp, yp;
    double A = MAP_MAX;
    
    // L1: y = 0.9A
    xp = xb + (0.9 * A - yb) * ((xt - xb) / (yt - yb));
    yp = 0.9 * A;

    if(((0.1 * A <= xp) && (xp <= 0.9 * A)) &&
    ((yb - 0.9 * A) * (yt - 0.9 * A) <= 0))
    {
      *xtt = xp;
      *ytt = yp;
      return;
    }

    // L2: y = 0.1A
    xp = xb + (0.1 * A - yb) * ((xt - xb) / (yt - yb));
    yp = 0.1 * A;

    if(((0.1 * A <= xp) && (xp <= 0.9 * A)) &&
    ((yb - 0.1 * A) * (yt - 0.1 * A) <= 0))
    {
      *xtt = xp;
      *ytt = yp;
      return;
    }

    // L3: x = 0.1A
    xp = 0.1 * A;
    yp = yb + (0.1 * A - xb) * ((yt - yb) / (xt - xb));

    if(((0.1 * A <= xp) && (xp <= 0.9 * A)) &&
    ((yb - 0.1 * A) * (yt - 0.1 * A) <= 0))
    {
      *xtt = xp;
      *ytt = yp;
      return;
    }

  }

  void GPSspace::GPS::Local_Map_Coordinate(double north_d,double east_d,double yaw_d, int *tx, int *ty)
  {
    double y1 =(refN - north_d)*(Lat_Dist) ; 
    double x1 = (refE - east_d)*(Long_Dist); 
    
    yaw_d = (3.142/180.0)*yaw_d;
    double x2 = x1*cos(yaw_d) -  y1*sin(yaw_d);
    double y2 = y1*cos(yaw_d) + x1*sin(yaw_d);

    x2 *= 1;
    y2 *= -1;

    truncate(x2*100, y2*100, tx, ty);
  }

void GPSspace::GPS::GPS_Init()
{
  string serName;
  
  if (mrpt::system::fileExists("./CONFIG_gps.ini"))
  {
    CConfigFile iniFile("./CONFIG_gps.ini");
    gps.loadConfig(iniFile, "GPS");
  }
  else
  {
    if (SERIAL_NAME.empty())
    {
      serName = GPS_COM;
    }
    else
    {
      serName = SERIAL_NAME;
    }

    // Set the laser serial port:
    gps.setSerialPortName(serName);
  }
}

void GPSspace::GPS::_GPS(double *north_d , double *east_d)
{
    FILE  *f= os::fopen("gps_log.txt","wt");
        fstream a_file;
        a_file.open("example.txt", ios::app| ios::in | ios::out);                    // Longitude and Latitude

        ofstream gps_data;
        gps_data.open("gps_data.txt");
  if (!f) return;

  CGenericSensor::TListObservations        lstObs;
  CGenericSensor::TListObservations::iterator   itObs;
        
  while (! mrpt::system::os::kbhit())
  {
    gps.doProcess();
    mrpt::system::sleep( 500 );

    gps.getObservations( lstObs );

    if (lstObs.empty())
    {
    }
    else
    {
      for (itObs=lstObs.begin();itObs!=lstObs.end();itObs++)
      {
             ASSERT_(itObs->second->GetRuntimeClass()==CLASS_ID(CObservationGPS));

             CObservationGPSPtr gpsData=CObservationGPSPtr(itObs->second);
                              // gpsData->GGA_datum;
                              // gpsData->GGA_datum.latitude_degrees;
                               //gpsData->GGA_datum.longitude_degrees;
                               //gps_data<<gpsData->GGA_datum.latitude_degrees;
                              //gpsData->dumpToConsole();
                              a_file.precision(12);   
                              a_file<<"  "<< gpsData->GGA_datum.latitude_degrees <<setprecision(12)<<"     "<<  gpsData->GGA_datum.longitude_degrees <<setprecision(12)<<endl;                       
                              //cout<<"DATA WRITTEN";
                              ///printf("Longitude_degrees-- %7.9lf\n",gpsData->GGA_datum.longitude_degrees);
                              //os::fprintf(f,byRows ? "%u ":"%u\n",static_cast<unsigned int>(*it));
                             
                                                              
                               //vectorToTextFile(gpsData->GGA_datum.latitude_degrees,"test_txt.txt",1,1);
                              *north_d = gpsData->GGA_datum.latitude_degrees;
                *east_d =  gpsData->GGA_datum.longitude_degrees;
                               
                break;
                          
      }
                         
                        //os::fclose(m);
                        //gps_data.close();
                        //fclose(m);
      lstObs.clear();
            break;          
    }
                                              ///printf("Longitude_degrees-- %7.9lf\n",gpsData->GGA_datum.longitude_degrees)
                //fclose(m);
  }
    a_file.close();
      os::fclose(f);
}

}
