#ifndef _PLANNER_H_
#define _PLANNER_H_
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "../../eklavya2.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cvaux.hpp>
#include <opencv/cxcore.h>
#include <cv.h>
#include <highgui.h>
#include <iostream>
#include <time.h>
//using namespace cv;
//#define MATDATA(img,x,y,n) img.at<cv::Vec3b>(x,y)[n]
#define LEFT_CMD 0
#define RIGHT_CMD 1

extern char** local_map;
extern cv::Mat map_img;
extern int ol_overflow;
//extern geometry_msgs::Twist precmdvel;
extern int last_cmd;

namespace planner_space {

    typedef struct command {
        int left_velocity, right_velocity;
    } command;

    class Planner {
    public:
        //        static  ros::NodeHandle nh;
        //
        //
        //     static ros::Publisher vel_pub;

        static void loadPlanner();
        static geometry_msgs::Twist findPath(Triplet bot, Triplet target, cv::Mat map_img);
        static geometry_msgs::Twist findPathDT(Triplet bot, Triplet target, cv::Mat map_img);
        static void finBot();
    };
}

#endif
