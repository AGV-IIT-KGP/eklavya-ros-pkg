#include <stdio.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "../../eklavya2.h"
#include "planner.h"

char** local_map;

void *planner_thread(void *arg) {
  int iterations = 0;
  double heading;
  Triplet my_bot_location;
  Triplet my_target_location;
  
  int argc;
  char *argv[0];
  
  local_map = new char*[MAP_MAX];
  for (int i = 0; i < MAP_MAX; i++) {
    local_map[i] = new char[MAP_MAX];
  }
  
  //ros::init(argc, argv, "planner");
  //ros::NodeHandle n;
  
  //ros::Publisher planner_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  
  printf("Initiating Planner\n");
  planner_space::Planner::loadPlanner();
  printf("\tPlanner Initiated\n");

  //while(1) {
    pthread_mutex_lock(&bot_location_mutex);
    my_bot_location = bot_location; // Bot
    pthread_mutex_unlock(&bot_location_mutex);
    
    my_bot_location.x = 500; my_bot_location.y = 100; my_bot_location.z = 90;
    pthread_mutex_lock(&target_location_mutex);
    my_target_location = target_location; // Target
    pthread_mutex_unlock(&target_location_mutex);
        
    my_target_location.x = 500; my_target_location.y = 900; my_target_location.z = 90;    

    pthread_mutex_lock(&map_mutex);
    for(int i = 0; i < MAP_MAX; i++) {
      for(int j = 0; j < MAP_MAX; j++) {
        local_map[i][j] = global_map[i][j];
      }
    }
    pthread_mutex_unlock(&map_mutex);
    
    vector<Triplet> path = planner_space::Planner::findPath(my_bot_location, my_target_location);
    
    for(int i = 0; i < path.size(); i++) {
      cout << "{ " << path[i].x << " , " << path[i].y << " , " << path[i].z << " }" << endl;
    }
    
    iterations++;
  //}
}

