#include <stdio.h>

#include "ros/ros.h"

#include <eklavya_encoder/encoder.h>

using namespace std;

int main(int argc, char **argv) {
    encoder_space::Encoder encoder(argc, argv);
    encoder_space::EncoderData encoder_data;
    
    const string publisher_name = "encoder";
    
    ros::init(argc, argv, "encoder");
    
    ros::NodeHandle n;
    ros::Publisher encoder_publisher = n.advertise<eklavya_encoder::Encoder_Data>(publisher_name, (uint32_t) 30, true);
    eklavya_encoder::Encoder_Data message;
    
    ROS_INFO("Encoder node initialized...");
    
    while (ros::ok()) {
        /* Fetch data from Shaft Encoder and load it in local vars */
        encoder_data = encoder.fetchEncoderData();
        
        ROS_INFO("before waitForService");
        ros::service::waitForService("motor_speed");
        ROS_INFO("after waitForService");
        /* Generate message using data */
        message.left_count = encoder_data.leftCount;
        message.right_count = encoder_data.rightCount;
        
        /* Publish message to the topic */
        ROS_INFO("Publishing encoder data : left = %d, right = %d", message.left_count, message.right_count);
        encoder_publisher.publish(message);

        usleep(10);
    }

    return 0;
}

