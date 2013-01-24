
#include <algorithm>
#include <assert.h>

//#include <eklavya_bringup/diffdrive_robot_controller.h>


#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <boost/bind.hpp>
#include "serial_lnx.h"
#include <string.h>
#define UART_COMM_PORT  "/dev/ttyUSB0"
#define UART_BAUD_RATE 19200

using namespace std;
Tserial* p; //Serial definition
//Declaration for the bot
int wheelSpeed[2];
int scale=100;
double x_;
double rot_;
ros::Time current_time, last_time;
std::string topicName,port,cmd_topic_name;
double wheelSeparation;
double wheelDiameter;
double torque;
int baud_rate;
bool dummy;
void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);
enum
{
    RIGHT,
    LEFT,
};
boost::mutex lock;




void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg)
{
    lock.lock();
    ROS_INFO("cmdVelCallback: cmd_msg->linear.x %lf cmd_msg->angular.z %lf\n",cmd_msg->linear.x,cmd_msg->angular.z);
    last_time=ros::Time::now();
    x_ = cmd_msg->linear.x;
    rot_ = cmd_msg->angular.z;
    double vr, va;

    vr = x_; //myIface->data->cmdVelocity.pos.x;
    va = rot_; //myIface->data->cmdVelocity.yaw;

    wheelSpeed[LEFT] = ((double)vr +(double) va * (double)wheelSeparation / 2.0)*scale;
    wheelSpeed[RIGHT] = ((double)vr - (double)va * (double)wheelSeparation / 2.0)*scale;
    ROS_INFO("Sending w %d %d : \n",wheelSpeed[LEFT],wheelSpeed[RIGHT]);
	if(!dummy)
	{
         
	    p->sendChar('w');
	   
	    usleep(100);

	    p->sendChar('0' + wheelSpeed[LEFT] / 10);
	    usleep(100);
	    p->sendChar('0' + wheelSpeed[LEFT] % 10);
	    usleep(100);
	    p->sendChar('0' + wheelSpeed[RIGHT] / 10);
	    usleep(100);
	    p->sendChar('0' + wheelSpeed[RIGHT] % 10);
	    
	}
lock.unlock();
}

int main( int argc, char** argv) {

    ros::init(argc, argv, "diffdrive_robot_controller");

    ros::NodeHandle rosnode_robot_controller("diffdrive_robot_controller");
    rosnode_robot_controller.param("wheelSeparation", wheelSeparation, 0.55000000);
    rosnode_robot_controller.param("dummy", dummy, false);
    rosnode_robot_controller.param("port", port, string("/dev/ttyUSB0"));
    rosnode_robot_controller.param("baud_rate", baud_rate, 19200);
    rosnode_robot_controller.param("wheelDiameter", wheelDiameter, 0.30550000);
    rosnode_robot_controller.param("cmd_topic_name", cmd_topic_name, string("/cmd_vel"));
    ROS_INFO("\nstarting diffdrive robot controller with wheel separation: %lf", wheelSeparation);
    ros::Subscriber sub = rosnode_robot_controller.subscribe(cmd_topic_name, 1000, cmdVelCallback);



    current_time = ros::Time::now();

	if(!dummy)
		{
  		  ROS_INFO("Connecting to serial port %s %d ",port.c_str(),baud_rate);
	    p = new Tserial();
	    p->connect("/dev/ttyUSB0", baud_rate, spNONE);
	    usleep(100);
	    p->sendChar('w');

	    usleep(100);
	}
    ros::Rate loop_rate(20.0);
    while (rosnode_robot_controller.ok()) {

        current_time = ros::Time::now();
        //double dt = (current_time - last_time).toSec();
        ros::spinOnce();

        loop_rate.sleep();
    }

}
