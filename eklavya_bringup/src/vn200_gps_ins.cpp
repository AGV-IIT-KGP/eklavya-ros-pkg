#include <iostream>
#include <unistd.h>
#include <ros/ros.h>
#include "serial_lnx.h"
#include <string.h>
#define UART_COMM_PORT  "/dev/serial/by-id/usb-FTDI_USB-RS232_Cable_FTUTUVO5-if00-port0"
#define UART_BAUD_RATE 115200

using namespace std;

Tserial* p; 

int main( int argc, char** argv) {
  double gpsTime;
	unsigned short gpsWeek, status;
	//VnVector3 ypr, latitudeLognitudeAltitude, nedVelocity;
	float attitudeUncertainty, positionUncertainty, velocityUncertainty;
  double x,y,z,a,f,b,e2,Nphi,latitude,longitude,altitude;

  //ros::init(argc, argv, "vn200_gps_ins");
  //ros::NodeHandle vn200("vn200_gps_ins");
  //ros::Publisher gps = vn200.advertise<sensor_msgs::NavSatFix>("fix", 1);
  
  //ros::Rate loop_rate(10);
  
  //p = new Tserial();
  //p->connect(UART_COMM_PORT, UART_BAUD_RATE, spNONE);
  //usleep(100);
  //char c;
  
  //cout << "FIRST" << endl;
  
  //while((c = p->getChar()) != '$') {
    ////printf("%c", c);
    //usleep(100);
  //}
  
  //cout << "SECOND" << endl;
  
  //while((c = p->getChar()) == '$') {
    //printf("%c", c);
    //usleep(100);
  //}
  
  char data[1000];
  data[0] = '\0';
  int fd = open(UART_COMM_PORT, O_RDWR | O_NOCTTY | O_NDELAY | O_FSYNC );
  
  while(strcmp(data, "$VNINS") != 0) {
    read(fd, data, 6);
    data[6] = '\0';
  }
  
  int n = read(fd, data, 10);
  data[10] = '\0';
  printf("%x\n", data);
  
  /*while(ros::ok()) {
    //sensor_msgs::NavSatFix fix;
    //fix.header.stamp = ros::Time::now();
    
    
    //gps.publish(fix);
    
    loop_rate.sleep();
  }*/
}
