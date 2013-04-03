#include "../../eklavya2.h"
#include <cv_bridge/CvBridge.h>
#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <stdexcept>
#include <cvblob.h>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/core/core_c.h>
#include <sensor_msgs/Image.h>

#ifndef LANE_DATA_H
#define	LANE_DATA_H

class LaneDetection {
public:
    void markLane(const sensor_msgs::ImageConstPtr& image);
    IplImage* colorBasedLaneDetection(IplImage* frame_in, int maxvalue_B, int maxvalue_G, int maxvalue_R, int minvalue_B, int minvalue_G, int minvalue_R);
    void applyHoughTransform(IplImage* img, IplImage *dst, int vote, int length, int merge);
    IplImage* joinResult(IplImage* color_gray, IplImage* hough_gray);
    void initializeLaneVariables(IplImage *img);
};

using namespace cvb;

#endif
