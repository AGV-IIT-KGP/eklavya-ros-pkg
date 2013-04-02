#include "gps.h"

namespace gps_space {

    void GPS::updatePose(const geometry_msgs::Pose::ConstPtr _pose) {
        pthread_mutex_lock(&pose_mutex);

        pose.position.x = _pose->position.x;
        pose.position.y = _pose->position.y;

        pthread_mutex_unlock(&pose_mutex);
    }
}
