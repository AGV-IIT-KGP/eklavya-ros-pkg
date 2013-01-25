#include <stdio.h>
#include "../../eklavya2.h"
#include "PathPlanner.h"

void *planner_thread(void *arg) {
  int iterations = 0;
  double heading;
  Triplet my_target_location;
  
  int argc;
  char *argv[0];
  
  ros::init(argc, argv, "planner");
  ros::NodeHandle n;
  
  ros::Publisher planner_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  
  while(ros::ok()) {
    pthread_mutex_lock(&target_location_mutex);
    my_target_location = target_location; // Target
    pthread_mutex_unlock(&target_location_mutex);
        
    pthread_mutex_lock(&pose_mutex);
    heading = pose.orientation.z; // Yaw
    pthread_mutex_unlock(&pose_mutex);
    
    Nav::command cmd = Nav::NavCore::navigate(my_target_location, iterations, heading);
    
    double scale = 100;
    double w = 0.55000000;
    geometry_msgs::Twist cmd_msg;
    cmd_msg.linear.x = (cmd.left_velocity + cmd.right_velocity) / (2 * scale);
    cmd_msg.angular.z = (cmd.left_velocity - cmd.right_velocity) / (w * scale);
    
    planner_pub.publish(cmd_msg);

    iterations++;
  }
}

