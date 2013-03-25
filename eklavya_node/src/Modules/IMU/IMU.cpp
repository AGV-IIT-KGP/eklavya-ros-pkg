/* 
 * File:   Imu_main.cpp
 * Author: Samuel, Nikunj, Atul
 *
 */

#include "IMU.h"

namespace IMUspace {

    void IMUspace::IMU::update_yaw(const std_msgs::Float32& _yaw) {
        pthread_mutex_lock(&pose_mutex);

        pose.orientation.z = _yaw.data;
        //cout << "[IMU] [YAW] : " << pose.orientation.z << endl;

        pthread_mutex_unlock(&pose_mutex);
    }

    void IMUspace::IMU::update_pose(const sensor_msgs::Imu& imu_msg) {
        tf::Quaternion tmp_;

        tf::quaternionMsgToTF(imu_msg.orientation, tmp_);

        tfScalar yaw, pitch, roll;
        tf::Matrix3x3(tmp_).getRPY(roll, pitch, yaw);

        //cout << "[IMU] [RPY] : (" << roll << ", " << pitch << ", " << yaw << ")" << endl;

        pthread_mutex_lock(&pose_mutex);

        pose.orientation.x = (double) roll * 180 / 3.14;
        pose.orientation.y = (double) pitch * 180 / 3.14;
        pose.orientation.z = (double) yaw * 180 / 3.14; // Yaw

        //cout << "[IMU] [YAW] : " << yaw << endl;

        pthread_mutex_unlock(&pose_mutex);
    }
}

