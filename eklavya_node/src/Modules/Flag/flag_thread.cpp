#include "flag_data.h"
#include "ros/ros.h"
#include <cvblob.h>
#include "../../ExternalLib/cvBlob/BlobResult.h"
#include <opencv2/legacy/blobtrack.hpp>
#include <iostream>
#include <vector>
#include <algorithm>


void *flag_thread(void *arg) 
{
      ros::NodeHandle flag_node;
      
      flag_space::FlagDetection flag_d;
      image_transport::ImageTransport it(flag_node);
      image_transport::Subscriber sub = it.subscribe("camera/flag_image", 0, &flag_space::FlagDetection::markFlag, &flag_d);
      ros::Rate loop_rate(LOOP_RATE);
      ROS_INFO("Flag thread started");
      while(ros::ok()) 
        {
                ros::spinOnce();
                loop_rate.sleep();
        }
      
      ROS_INFO("Flag code exiting");
      
      return NULL;
}
