<<<<<<< HEAD
#include <stdio.h>
=======
>>>>>>> 5b6c52d7db233097e1f897db6dcb4b9a52417cae
#include "IMU.h"

void *imu_thread(void *arg) {
    ros::NodeHandle imu_node;
    ros::Subscriber sub = imu_node.subscribe("/yaw", 1, IMUspace::IMU::update_yaw);
    ros::Rate loop_rate(LOOP_RATE);

<<<<<<< HEAD
=======

>>>>>>> 5b6c52d7db233097e1f897db6dcb4b9a52417cae
    ROS_INFO("Started IMU thread");
    
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return NULL;
}

