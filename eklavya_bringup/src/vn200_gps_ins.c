#include <stdio.h>
#include <unistd.h>
#include "vectornav.h"
#include <ros/ros.h>

using namespace std;

const char* const COM_PORT = "/dev/serial/by-id/usb-FTDI_USB-RS232_Cable_FTUTUVO5-if00-port0";
const int BAUD_RATE = 115200;
Vn200 vn200;

int main( int argc, char** argv) {
  double gpsTime;
	unsigned short gpsWeek, status;
	VnVector3 ypr, latitudeLognitudeAltitude, nedVelocity;
	float attitudeUncertainty, positionUncertainty, velocityUncertainty;
  double x,y,z,a,f,b,e2,Nphi,latitude,longitude,altitude;

  ros::init(argc, argv, "vn200_gps_ins");
  ros::NodeHandle vn200("vn200_gps_ins");
  //ros::Publisher gps = vn200.advertise<sensor_msgs::NavSatFix>("fix", 1);
  
  ros::Rate loop_rate(10);
  
  while(ros::ok()) {
    //sensor_msgs::NavSatFix fix;
    //fix.header.stamp = ros::Time::now();
    
    
    //gps.publish(fix);
    
    loop_rate.sleep();
  }
}
