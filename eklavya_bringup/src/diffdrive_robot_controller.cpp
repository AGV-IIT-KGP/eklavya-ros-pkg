/*
    Copyright (c) 2010, Daniel Hewlett, Antons Rebguns
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
        * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
        * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
        * Neither the name of the <organization> nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY Antons Rebguns <email> ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL Antons Rebguns <email> BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#include <algorithm>
#include <assert.h>

#include <eklavya_bringup/diffdrive_robot_controller.h>


#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <boost/bind.hpp>
#include "../../Utils/SerialPortLinux/serial_lnx.h"



enum
{
  RIGHT,
  LEFT,
};

// Constructor
DiffDrivePlugin::DiffDrivePlugin()
{
}

// Destructor
DiffDrivePlugin::~DiffDrivePlugin()
{
  delete rosnode_;
  delete transform_broadcaster_;
}

// Load the controller
void main(int argc, char** argv)
{
  

  wheelSpeed[RIGHT] = 0;
  wheelSpeed[LEFT] = 0;

  x_ = 0;
  rot_ = 0;
  alive_ = true;
  double wheelSeparation,wheelDiameter;
  char* port, cmd_topic_name;
  int baud_rate;
  
  // Initialize the ROS node and subscribe to cmd_vel
  ros::init(argc, argv, "diffdrive_robot_controller", ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
  rosnode_ = new ros::NodeHandle("diffdrive_robot_controller");
  rosnode_.param("wheelSeparation", wheelSeparation, 0.55000000);
  rosnode_.param("port", port, "/dev/ttyUSB0");
  rosnode_.param("baud_rate", baud_rate, 19200);
  rosnode_.param("wheelDiameter", wheeldiameter, 0.30550000);
  rosnode_.param("cmd_topic_name", cmd_topic_name, "cmd_vel");
  ROS_INFO("starting diffdrive robot controller with wheel separation: %s", wheelSeparation);
  p = new Tserial();
      p->connect(BOT_COM_PORT, BOT_BAUD_RATE, spNONE);
      usleep(100);

      p->sendChar('w');
      usleep(100);
 // tf_prefix_ = tf::getPrefixParam(*rosnode_);
  //transform_broadcaster_ = new tf::TransformBroadcaster();

  // ROS: Subscribe to the velocity command topic (usually "cmd_vel")
  ros::SubscribeOptions so =
      ros::SubscribeOptions::create<geometry_msgs::Twist>(cmd_topic_name, 1,
                                                          boost::bind(&DiffDrivePlugin::cmdVelCallback, this, _1),
                                                          ros::VoidPtr(), &queue_);
  sub_ = rosnode_->subscribe(so);
  //pub_ = rosnode_->advertise<nav_msgs::Odometry>("odom", 1);
/*
  // Initialize the controller
  // Reset odometric pose
  odomPose[0] = 0.0;
  odomPose[1] = 0.0;
  odomPose[2] = 0.0;

  odomVel[0] = 0.0;
  odomVel[1] = 0.0;
  odomVel[2] = 0.0;
*/
  // start custom queue for diff drive
 // this->callback_queue_thread_ = boost::thread(boost::bind(&DiffDrivePlugin::QueueThread, this));

  // listen to the update event (broadcast every simulation iteration)
    ros::Rate loop_rate(10);
  while(ros::ok){
  DiffDrivePlugin::UpdateChild();
    ros::spinOnce();

    loop_rate.sleep();
  }
  
  
  alive_ = false;
  //queue_.clear();
  //queue_.disable();
  rosnode_->shutdown();
  callback_queue_thread_.join();
 
   p->sendChar(' ');
      usleep(100);

      p->disconnect();
      usleep(100);
}

// Update the controller
void DiffDrivePlugin::UpdateChild()
{
  // TODO: Step should be in a parameter of this function
  double wd, ws;
  double d1, d2;
  double dr, da;
  
  GetPositionCmd();

  wd = wheelDiameter;
  
  ws = wheelSeparation;
/*
  // Distance travelled by front wheels
  d1 = stepTime * wd / 2 * joints[LEFT]->GetVelocity(0);
  d2 = stepTime * wd / 2 * joints[RIGHT]->GetVelocity(0);

  dr = (d1 + d2) / 2;
  da = (d1 - d2) / ws;

  // Compute odometric pose
  odomPose[0] += dr * cos(odomPose[2]);
  odomPose[1] += dr * sin(odomPose[2]);
  odomPose[2] += da;

  // Compute odometric instantaneous velocity
  odomVel[0] = dr / stepTime;
  odomVel[1] = 0.0;
  odomVel[2] = da / stepTime;

  joints[LEFT]->SetVelocity(0, wheelSpeed[LEFT] / (wheelDiameter / 2.0));
  joints[RIGHT]->SetVelocity(0, wheelSpeed[RIGHT] / (wheelDiameter / 2.0));

  joints[LEFT]->SetMaxForce(0, torque);
  joints[RIGHT]->SetMaxForce(0, torque);

  //write_position_data();
  publish_odometry();*/

	p->sendChar('w');
      usleep(100);

      p->sendChar('0' + +wheelSpeed[LEFT] / 10);
      usleep(100);
      p->sendChar('0' + +wheelSpeed[LEFT] % 10);
      usleep(100);
      p->sendChar('0' + wheelSpeed[RIGHT] / 10);
      usleep(100);
      p->sendChar('0' + wheelSpeed[RIGHT] % 10);
}



void DiffDrivePlugin::GetPositionCmd()
{
  lock.lock();

  double vr, va;

  vr = x_; //myIface->data->cmdVelocity.pos.x;
  va = rot_; //myIface->data->cmdVelocity.yaw;

  //std::cout << "X: [" << x_ << "] ROT: [" << rot_ << "]" << std::endl;

  wheelSpeed[LEFT] = vr + va * wheelSeparation / 2.0;
  wheelSpeed[RIGHT] = vr - va * wheelSeparation / 2.0;
  
  lock.unlock();
}

void DiffDrivePlugin::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg)
{
  lock.lock();

  x_ = cmd_msg->linear.x;
  rot_ = cmd_msg->angular.z;

  lock.unlock();
}


/*
void DiffDrivePlugin::publish_odometry()
{
  ros::Time current_time = ros::Time::now();
  std::string odom_frame = tf::resolve(tf_prefix_, "odom");
  std::string base_footprint_frame = tf::resolve(tf_prefix_, "base_footprint");

  // getting data for base_footprint to odom transform
  math::Pose pose = this->parent->GetState().GetPose();

  btQuaternion qt(pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w);
  btVector3 vt(pose.pos.x, pose.pos.y, pose.pos.z);

  tf::Transform base_footprint_to_odom(qt, vt);
  transform_broadcaster_->sendTransform(tf::StampedTransform(base_footprint_to_odom,
                                                             current_time,
                                                             odom_frame,
                                                             base_footprint_frame));

  // publish odom topic
  odom_.pose.pose.position.x = pose.pos.x;
  odom_.pose.pose.position.y = pose.pos.y;

  odom_.pose.pose.orientation.x = pose.rot.x;
  odom_.pose.pose.orientation.y = pose.rot.y;
  odom_.pose.pose.orientation.z = pose.rot.z;
  odom_.pose.pose.orientation.w = pose.rot.w;

  math::Vector3 linear = this->parent->GetWorldLinearVel();
  //odom_.twist.twist.linear.x = linear.x;
 // odom_.twist.twist.linear.y = linear.y;
 // odom_.twist.twist.angular.z = this->parent->GetWorldAngularVel().z;

  odom_.header.stamp = current_time;
  odom_.header.frame_id = odom_frame;
  odom_.child_frame_id = base_footprint_frame;
  
  double pose_cov[36] = { 1e-3, 0, 0, 0, 0, 0,
                          0, 1e-3, 0, 0, 0, 0,
                          0, 0, 1e6, 0, 0, 0,
                          0, 0, 0, 1e6, 0, 0,
                          0, 0, 0, 0, 1e6, 0,
                          0, 0, 0, 0, 0, 1e3};

  memcpy( &odom_.pose.covariance[0], pose_cov, sizeof(double)*36 );
  memcpy( &odom_.twist.covariance[0], pose_cov, sizeof(double)*36 );

  odom_.twist.twist.linear.x = 0;
  odom_.twist.twist.linear.y = 0;
  odom_.twist.twist.linear.z = 0;

  odom_.twist.twist.angular.x = 0;
  odom_.twist.twist.angular.y = 0;
  odom_.twist.twist.angular.z = 0;

  pub_.publish(odom_);
}
*/

}
