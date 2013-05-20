#include <ros/ros.h>

#include <eklavya_roboteq/GetSpeed.h>

#include <eklavya_encoder/encoder.h>

ros::NodeHandle *n;
ros::ServiceClient client;
eklavya_roboteq::GetSpeed *srv;

namespace encoder_space {

    Encoder::Encoder(int argc, char **argv) {
        ros::init(argc, argv, "motor_speed_client");
        if (argc != 1) {
            ROS_INFO("usage: motor_speed_client");
        }

        n = new ros::NodeHandle();

        client = n->serviceClient<eklavya_roboteq::GetSpeed > ("motor_speed");

        srv = new eklavya_roboteq::GetSpeed();
    }

    EncoderData Encoder::fetchEncoderData() {
        EncoderData returnValue;

        returnValue.leftCount = 0;
        returnValue.rightCount = 0;

        if (client.call(*srv)) {
            ROS_INFO("Encoder data : left speed = %ld, right speed = %ld", (long int) srv->response.left_speed, (long int) srv->response.right_speed);
            returnValue.leftCount = srv->response.left_speed;
            returnValue.rightCount = srv->response.right_speed;
        } else {
            ROS_ERROR("Failed to call service get speed");
        }

        return returnValue;
    }

}
