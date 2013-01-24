#include <stdio.h>

#include "opencv/cv.h"
#include "opencv2/highgui/highgui.hpp"
#include "diagnostics.h"
#include "../../eklavya2.h"

namespace diagnostics_space {
  void diagnostics_space::Diagnostics::plotMap() {
    IplImage *image = cvCreateImage(cvSize(MAP_MAX, MAP_MAX), IPL_DEPTH_8U, 3);
    
    for (int i = 0; i < MAP_MAX; i++) {
      uchar* ptr = (uchar *)(image->imageData + i * image->widthStep);
      for (int j = 0; j < MAP_MAX; j++) {
        if (global_map[j][MAP_MAX - i - 1] > 0) {
          ptr[3 * j] = 0;
          ptr[3 * j + 1] = 0;
          ptr[3 * j + 2] = 0;
        } else {
          ptr[3 * j] = 200;
          ptr[3 * j + 1] = 200;
          ptr[3 * j + 2] = 200;
        }
      }
    }
    
    cvShowImage("Map", image);
    cvWaitKey(1);
    cvReleaseImage(&image);
  }
  
  void diagnostics_space::Diagnostics::printPose() {
    printf("Pose: [Position: (%lf, %lf, %lf); Orientation: (%lf, %lf, %lf)]\n", pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  }
  
  void diagnostics_space::Diagnostics::printLatLong() {
    printf("LatLong: (%lf, %lf)\n", lat_long.latitude, lat_long.longitude);
  }
  
  void diagnostics_space::Diagnostics::printOdom() {
    printf("Odom: (%lf, %lf)\n", odom.left_velocity, odom.right_velocity);
  }
  
  void diagnostics_space::Diagnostics::printBotLocation() {
    printf("Bot Location: (%lf, %lf, %lf)\n", bot_location.x, bot_location.y, bot_location.z);
  }

  void diagnostics_space::Diagnostics::printTargetLocation() {
    printf("Target Location: (%lf, %lf, %lf)\n", target_location.x, target_location.y, target_location.z);
  }

}
