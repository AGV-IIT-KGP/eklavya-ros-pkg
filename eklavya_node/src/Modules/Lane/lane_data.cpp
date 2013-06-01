#include "lane_data.h"
#include <cvblob.h>
#include <math.h>

using namespace cvb;

#define DEBUG 0
#define EXPANSION 60

sensor_msgs::CvBridge bridge;
static int iter = 0;
IplImage *blue_img;
IplImage *green_img;
IplImage *filter_img;
IplImage *warp_img;
IplImage* img;
IplConvKernel *ker1;
CvPoint2D32f srcQuad[4], dstQuad[4];
CvMat* warp_matrix = cvCreateMat(3, 3, CV_32FC1);

int vote = 16, length = 30, mrg = 100;
int k = 250;

IplImage* LaneDetection::colorBasedLaneDetection(IplImage *img) {
    double mean,std_dev;
    int height = img->height;
    int width = img->width;
    //Calculating Mean of the image
    double total = 0;
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            total += CV_IMAGE_ELEM(img, unsigned char, i, j);
        }
    }
    mean = (total/(height * width));
    //Calculating Standard Deviation of the image
    double var = 0;
    for (int i=0; i<height; i++)
    {
        for (int j=0; j<width; j++)
        {
            var += pow(CV_IMAGE_ELEM(img, unsigned char, i, j)-mean, 2);
        }
    }
    var /= (height * width);
    std_dev = sqrt(var);
    cvThreshold(img, img, mean+((k*std_dev)/100.0), 255, CV_THRESH_BINARY);
    //Morphological Operation to reduce noise
    IplConvKernel *convKernel = cvCreateStructuringElementEx(5, 5, 2, 2, CV_SHAPE_RECT);
    IplImage* simg=cvCloneImage(img);
    cvMorphologyEx(img, simg, NULL, convKernel, CV_MOP_OPEN);
    cvNot(simg, img);
    cvMorphologyEx(img, simg, NULL, convKernel, CV_MOP_OPEN);
    cvNot(simg, img);
    return img;
}

IplImage* LaneDetection::applyHoughTransform(IplImage* img) {
    CvSeq* lines;
    CvMemStorage* storage = cvCreateMemStorage(0);
    int i;    
    lines = cvHoughLines2(img, storage, CV_HOUGH_PROBABILISTIC, 1, CV_PI / 180, vote, length, mrg);    
    cvSetZero(img);
    int n = lines->total;
    for (i = 0; i < n; i++) 
    {
        CvPoint* line = (CvPoint*) cvGetSeqElem(lines, i);
        cvLine(img, line[0], line[1], CV_RGB(255, 255, 255), 5);
    }
    cvReleaseMemStorage(&storage);
    return img;
}

void LaneDetection::initializeLaneVariables(IplImage *input_frame) {
    blue_img = cvCreateImage(cvGetSize(input_frame), input_frame->depth, 1);
    green_img = cvCreateImage(cvGetSize(input_frame), input_frame->depth, 1);
    filter_img = cvCreateImage(cvGetSize(input_frame), input_frame->depth, 1);
    warp_img = cvCreateImage(cvSize(MAP_MAX, MAP_MAX), 8, 1);
    ker1 = cvCreateStructuringElementEx(3, 3, 1, 1, CV_SHAPE_ELLIPSE);
    //Destination variables
    int widthInCM = 100, h1 = 220, h2 = 295; //width and height of the lane. width:widthoflane/scale;
    
    srcQuad[0].x = (float) 134; //src Top left
    srcQuad[0].y = (float) 166;
    srcQuad[1].x = (float) 432; //src Top right
    srcQuad[1].y = (float) 170;
    srcQuad[2].x = (float) 62; //src Bottom left
    srcQuad[2].y = (float) 362;
    srcQuad[3].x = (float) 488; //src Bot right
    srcQuad[3].y = (float) 354;
    
    dstQuad[0].x = (float) (500 - widthInCM / (2)); //dst Top left
    dstQuad[0].y = (float) (999 - h2);
    dstQuad[1].x = (float) (500 + widthInCM / (2)); //dst Top right
    dstQuad[1].y = (float) (999 - h2);
    dstQuad[2].x = (float) (500 - widthInCM / (2)); //dst Bottom left
    dstQuad[2].y = (float) (999 - h1);
    dstQuad[3].x = (float) (500 + widthInCM / (2)); //dst Bot right
    dstQuad[3].y = (float) (999 - h1);

    cvGetPerspectiveTransform(dstQuad, srcQuad, warp_matrix);
}

void populateLanes(IplImage *img) {
    int i, j;
    pthread_mutex_lock(&camera_map_mutex);
    for (i = 0; i < img->height; i++) {
        uchar *data = (uchar *) (img->imageData + (MAP_MAX - i) * img->widthStep);
        for (j = 0; j < img->width; j++) {
            camera_map[j][i] = data[j];
        }
    }
    pthread_mutex_unlock(&camera_map_mutex);
}

void LaneDetection::markLane(const sensor_msgs::ImageConstPtr& image) {
    try {
        img = bridge.imgMsgToCv(image, "bgr8");
        cvWaitKey(WAIT_TIME);
    } catch (sensor_msgs::CvBridgeException& e) {
        ROS_ERROR("ERROR IN CONVERTING IMAGE!!!");
    }
    //Displaying the original image
    if (DEBUG) {
        cvShowImage("OriginalImage", img);
        cvWaitKey(WAIT_TIME);
    }
    //Initializing the required image variables
    if (iter == 0) {
        initializeLaneVariables(img);
    }
    iter++;
    //Enhancing the lanes
    cvSplit(img, blue_img, green_img, NULL, NULL);
    cvConvertScale(green_img, green_img, 0.5, 0);
    cvSub(blue_img, green_img, filter_img);
    cvConvertScale(filter_img, filter_img, 2, 0);
    //Displying the filtered image
//    if(DEBUG)
//    {
//        cvShowImage("Filtered Image", filter_img);
//        cvWaitKey(WAIT_TIME);
//    }
    //Adaptive Thresholding
    filter_img = colorBasedLaneDetection(filter_img);
    //Displying thresholded image
//    if(DEBUG)
//    {
//        cvShowImage("Thresholded Image", filter_img);
//        cvWaitKey(WAIT_TIME);
//    }
    //Finding Hough Lines
    filter_img = applyHoughTransform(filter_img);
    //Displaying hough lanes
//    if(DEBUG)
//    {
//        cvShowImage("Hough Image", filter_img);
//        cvWaitKey(WAIT_TIME);
//    }
    //Inverse Perspective Transform
    cvWarpPerspective(filter_img, warp_img, warp_matrix, CV_INTER_LINEAR | CV_WARP_INVERSE_MAP | CV_WARP_FILL_OUTLIERS);
    //Displaying Lane Map
    if (DEBUG) {
        cvShowImage("Lane Map", warp_img);
        cvWaitKey(WAIT_TIME);
    }
    cvDilate(warp_img, warp_img, ker1, EXPANSION);
    populateLanes(warp_img);
}
