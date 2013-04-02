#ifndef _IMU_H
#define _IMU_H

#include "../../eklavya2.h"
#include "tf/transform_broadcaster.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Float32.h"

#ifndef TF_MATRIX3x3_H
typedef btScalar tfScalar;
namespace tf {
    typedef btMatrix3x3 Matrix3x3;
}
#endif

namespace IMUspace {

    class IMU {
    public:
        static void update_pose(const sensor_msgs::Imu&);
        static void update_yaw(const std_msgs::Float32& _yaw);
    };
}

#endif
