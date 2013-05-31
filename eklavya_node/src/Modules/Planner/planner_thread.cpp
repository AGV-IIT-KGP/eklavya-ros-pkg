#include "planner.h"

#include <sstream>
//#define FPS_TEST

char **local_map;
//IplImage *map_img;

int ol_overflow;
//geometry_msgs::Twist precmdvel;
int last_cmd;

void *planner_thread(void *arg) {
    Triplet my_bot_location;
    Triplet my_target_location;
    //     precmdvel.linear.x = 0;
    //        precmdvel.linear.y = 0;
    //        precmdvel.linear.z = 0;
    //        precmdvel.angular.x = 0;
    //        precmdvel.angular.y = 0;
    //        precmdvel.angular.z = 0;
    ros::NodeHandle nh;
    ros::Publisher vel_pub;

    vel_pub = nh.advertise<geometry_msgs::Twist > ("cmd_vel", 1);

    cvNamedWindow("[PLANNER] Map", 0);

    //initializing local map
    local_map = new char*[MAP_MAX];
    for (int i = 0; i < MAP_MAX; i++) {
        local_map[i] = new char[MAP_MAX];
    }

    ROS_INFO("Initiating Planner");
    planner_space::Planner::loadPlanner();
    ROS_INFO("Planner Initiated");

    ROS_INFO("Waiting for Target");
    usleep(999999);
    usleep(999999);

#ifdef FPS_TEST
    int iterations = 0;
    time_t start = time(0);
#endif

    ros::Rate loop_rate(LOOP_RATE);
    geometry_msgs::Twist cmdvel;
    last_cmd = LEFT_CMD;

    while (ros::ok()) {
        cv::Mat map_img(MAP_MAX, MAP_MAX, CV_8UC1, cv::Scalar(0));

#ifdef FPS_TEST
        if (iterations > 1000) {
            time_t finish = time(0);
            double fps = (iterations + 0.0) / (finish - start);
            ROS_INFO("[INFO] Iterations: %d", iterations);
            ROS_INFO("[INFO] FPS: %lf", fps);
            break;
        }
        iterations++;
#endif

#ifdef FPS_TEST
        my_bot_location.x = 500;
        my_bot_location.y = 100;
        my_bot_location.z = 90;
#else
        pthread_mutex_lock(&bot_location_mutex);
        my_bot_location = bot_location; // Bot
        pthread_mutex_unlock(&bot_location_mutex);
#endif

#ifdef FPS_TEST
        srand(rand() * time(0));
        double randx = 500;
        double randy = 900;

        //randx = 100 + rand() % 800; randy = 900;

        my_target_location.x = randx;
        my_target_location.y = randy;
        my_target_location.z = 90;
#else
        pthread_mutex_lock(&target_location_mutex);
        my_target_location = target_location; // Target
        pthread_mutex_unlock(&target_location_mutex);
#endif

        pthread_mutex_lock(&global_map_mutex);
        for (int i = 0; i < MAP_MAX; i++) {
            for (int j = 0; j < MAP_MAX; j++) {
                map_img.at<uchar > (MAP_MAX - 1 - j, i) = global_map[i][j];
                local_map[i][j] = global_map[i][j];
            }
        }
        pthread_mutex_unlock(&global_map_mutex);
       // my_target_location.x = 500;
       // my_target_location.y = 900;
       // my_target_location.z = 90; 
       // my_bot_location.x = 500;
       // my_bot_location.y = 100;
       // my_bot_location.z = 90;
        ol_overflow = 0;
        cmdvel = planner_space::Planner::findPathDT(my_bot_location, my_target_location, map_img);
        if (ol_overflow == 1) {
            ol_overflow = 0;
            cmdvel = planner_space::Planner::findPath(my_bot_location, my_target_location, map_img);
        }
        
        vel_pub.publish(cmdvel);

        loop_rate.sleep();
    }

    ROS_INFO("Planner Exited");

    return NULL;
}
