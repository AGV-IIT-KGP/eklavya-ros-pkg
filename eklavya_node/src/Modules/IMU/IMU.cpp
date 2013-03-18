/* 
 * File:   Imu_main.cpp
 * Author: Samuel, Nikunj, Atul
 *
 */

#include <iostream>
#include <fcntl.h>
#include <stdio.h>
#include <termios.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include "IMU.h"
#include "random.h"
#include "../../Utils/SerialPortLinux/serial_lnx.h"
#include "../devices.h"
#include "../../eklavya2.h"
#include "std_msgs/Float32.h"

#define D 0.1
#define WAIT 50
#define VAR_X 5
#define VAR_Y 7
#define VAR_I 0.01

#define VERBOSE false
#define Lat_Dist 111119.99
#define Long_Dist 102796.84                  
#define REF_NORTH 22.31666666666667
#define REF_EAST                   

namespace IMUspace
{
  //Declarations for IMU
  //LPWSTR filename1 = L"COM9:";    ///////IMU///////

  /* Change to the baud rate of the port B2400, B9600, B19200, etc */
  #define UART_BAUD_RATE  57600

  /* Change to the serial port you want to use /dev/ttyUSB0, /dev/ttyS0, etc. */
  #define UART_COMM_PORT IMU_COM_PORT

  FILE *f1;
  volatile double yawIMU=0;
  int time1=0;
  int fd;
  struct termios options;
  Tserial* p;
  
  void IMUspace::IMU::initIMU()
  {
    //fd=open( UART_COMM_PORT , O_RDONLY | O_NOCTTY );
    //if (fd <0) 
    //{   
      //perror(UART_COMM_PORT );
      //exit(-1); 
    //}
    
    //char c;
    //p = new Tserial();
    
    //p->connect(UART_COMM_PORT, UART_BAUD_RATE, spNONE,VERBOSE);
    //while((c = p->getChar()) != '!');
    //p->disconnect();
  }

  float convertDataToVal(char *data)
  {
    float yaw = 0;
    if(data[0] == '!')
    {
      if((data[1] != '+') && (data[1] != '-'))
        yaw = 777;
      else if(data[5] == '.')
      {
        yaw = (data[2] - '0') * 100 +
              (data[3] - '0') * 10 +
              (data[4] - '0') * 1 +
              (data[6] - '0') * 0.1 +
              (data[7] - '0') * 0.01;
        if(data[1] == '-')
          yaw *= -1;
      }
      else
        yaw = 666;
    }
    else
      yaw = 555;

    return yaw;
  }

  float convertDataToVal2(char *data)
  {
    float yaw = 0;

    if((data[0] != '+') && (data[0] != '-'))
      yaw = 777;
    else if(data[4] == '.')
    {
      yaw = (data[1] - '0') * 100 +
            (data[2] - '0') * 10 +
            (data[3] - '0') * 1 +
            (data[5] - '0') * 0.1 +
            (data[6] - '0') * 0.01;
      if(data[0] == '-')
        yaw *= -1;
    }
    else
      yaw = 666;

    return yaw;
  }
  
  void IMUspace::IMU::getYaw(double *yawIMUc)
  {
    char data_in[10];
    int flag=1,i;
    char inp;
    int linefeed_count=0;
    int r;
    int val = 0;
    int count = 0;

    //p->connect(UART_COMM_PORT, UART_BAUD_RATE, spNONE,VERBOSE);
    //while(p->getChar() != '!');
    //p->getArray(data_in, 8);
    //p->disconnect();
/*
    while(flag)
    {
      p->getArray(data_in, 10);
      tcflush(fd, TCIFLUSH);
      sleep(1);
      count++;
      if(count>5)
        break;
    }
*/
    *yawIMUc = convertDataToVal2(data_in);
  }

  void IMUspace::IMU::closeIMU()
  {
   // p->disconnect();
  }
  
  void IMUspace::IMU::update_yaw(const std_msgs::Float32& _yaw) {
    pthread_mutex_lock(&pose_mutex);
    
    pose.orientation.z = _yaw.data;
    printf("[INFO] [YAW] %lf\n", pose.orientation.z);
    
    pthread_mutex_unlock(&pose_mutex);
  }
  
  void IMUspace::IMU::update_pose(const sensor_msgs::Imu& imu_msg) {
    tf::StampedTransform transform_;
    tf::Quaternion tmp_;

    tf::quaternionMsgToTF(imu_msg.orientation, tmp_);

    tfScalar yaw, pitch, roll;
    tf::Matrix3x3(tmp_).getRPY(roll, pitch, yaw);
    
    //printf("RPY: %lf %lf %lf\n", roll, pitch, yaw);
    
    pthread_mutex_lock(&pose_mutex);
    
    pose.orientation.x = (double) roll * 180 / 3.14;
    pose.orientation.y = (double) pitch * 180 / 3.14;
    pose.orientation.z = (double) yaw * 180 / 3.14; // Yaw
    
    pthread_mutex_unlock(&pose_mutex);
  }
}

