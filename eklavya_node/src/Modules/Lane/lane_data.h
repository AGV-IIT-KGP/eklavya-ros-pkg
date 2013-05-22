#include "../../eklavya2.h"
#include <cv_bridge/CvBridge.h>
#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <stdexcept>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/core/core_c.h>
#include <sensor_msgs/Image.h>

#ifndef LANE_DATA_H
#define	LANE_DATA_H

extern IplImage *show_img1;
extern IplImage *show_img2;
extern IplImage *show_img3;
extern IplImage *show_img4;


class LaneDetection {
public:
    void markLane(const sensor_msgs::ImageConstPtr& image);
    IplImage* colorBasedLaneDetection(IplImage* frame_in);
    IplImage* applyHoughTransform(IplImage* img);
    IplImage* joinResult(IplImage* color_gray, IplImage* hough_gray);
    void initializeLaneVariables(IplImage *img);
};

#endif
