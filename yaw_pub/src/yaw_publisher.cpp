#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include "ros/ros.h"
#include "serial_lnx.h"
#include "std_msgs/Float32.h"

#define UART_COMM_PORT 	"/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A700ewWv-if00-port0"
#define UART_BAUD_RATE  57600
#define Sleep(i) usleep(i)

#define VERBOSE false

using namespace std;

float convertDataToVal2(char *data) {
  float yaw = 0;

  if ((data[0] != '+') && (data[0] != '-')) {
    yaw = 777;
  } else if (data[4] == '.') {
    yaw = (data[1] - '0') * 100 +
          (data[2] - '0') * 10 +
          (data[3] - '0') * 1 +
          (data[5] - '0') * 0.1 +
          (data[6] - '0') * 0.01;
    if(data[0] == '-') {
      yaw *= -1;
    }
  } else {
    yaw = 666;
  }
  
  return yaw;
}

int main(int argc, char **argv) {
  Tserial* p;
  
  p = new Tserial();
  p->connect(UART_COMM_PORT, UART_BAUD_RATE, spNONE);
  
  ros::init(argc, argv, "yaw_publisher");
  
  ros::NodeHandle n;
  ros::Publisher yaw_pub = n.advertise<std_msgs::Float32>("yaw", 10);

  std_msgs::Float32 msg;

  while(ros::ok()) {
    char data_in[10];

    while(p->getChar() != '!');
    p->getArray(data_in, 8);
    
    msg.data = convertDataToVal2(data_in);
    yaw_pub.publish(msg);
  }
  
  p->disconnect();
  return 0;
}
